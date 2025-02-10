#!/usr/bin/perl

BEGIN {
    $ENV{LC_ALL} = "C";
    $ENV{LANG}   = "en_US.UTF-8";
};

use strict; use warnings;

use FindBin;
use lib "$FindBin::Bin/../lib/perl";

use Socket qw(inet_ntoa SOL_SOCKET SO_KEEPALIVE TCP_NODELAY TCP_CORK SO_SNDTIMEO SO_RCVTIMEO PF_INET SOCK_STREAM SO_SNDBUF SO_RCVBUF SOCK_DGRAM PF_UNIX SOCK_DGRAM inet_aton pack_sockaddr_in pack_sockaddr_un);
use Fcntl qw(:DEFAULT);
use POSIX ();
use Time::HiRes ();

BEGIN {
    # TZ set so localtime() doesn't hit /etc/localtime, keep original tzname
    $ENV{TZ} = readlink('/etc/localtime') =~ s|/usr/share/zoneinfo/||gr;
    POSIX::tzset();
};

# signal handler setup
$::MB_LOOP = 1;
$SIG{TERM} = $SIG{INT} = $SIG{HUP} = sub {
    $::MB_LOOP = 0;
    return;
};
# no broken pipe errors
$SIG{PIPE} = 'IGNORE';

$ENV{LOGGER_SYSLOG} //= 0;
$ENV{LOGGER_STDERR} //= 1;
$ENV{SINK}          //= ">&STDOUT";

{
    my $do_daemon = cfg("DAEMON");
    my $do_login  = cfg("LOGIN");
    my $do_write  = cfg("WRITE");
    my $do_read   = cfg("READ");
    my $action    = $do_daemon
        ?"daemon":$do_login
        ?"login" :$do_write
        ?"write" :$do_read
        ?"read"  :"cli";
    my $what = "do_".lc($action);
    no strict "refs";
    &$what();
}
exit;

sub do_cli {
    $0 = "modbus:cli";
    return do_daemon(0);
}

sub do_daemon {
    my ($run_wait_time) = @_;
    $0 = "modbus:poller";

    $run_wait_time //= cfg("loop_wait_time", 15);
    logger::log_debug("running ".($run_wait_time?"as a daemon, every $run_wait_time seconds":"once"));
    my $tgts = parse_cfg();
    eval {
        my $do_run = 1;
        while($::MB_LOOP and $do_run){
            # continously run, or just run once? This isn't a cron-like scheduler, just a wait
            if($run_wait_time){
                if($do_run > 1){
                    sleep $run_wait_time;
                } else {
                    $do_run = 2;
                }
            } else {
                $do_run = 0;
            }

            # per register, process
            my $max_attempt_cnt = cfg("retry_max", 1);
            my $mb_conns = {};
            foreach my $r_entry (@{$tgts}){
                last unless $::MB_LOOP;
                my ($tgt_modbus_peer, $unit_id, $register, $mb_msg, $value, $sleepy) = @$r_entry;
                my $mb_fh;
                eval {
                    # initiate a connection (but cache it)
                    $mb_fh = $mb_conns->{$tgt_modbus_peer} //= do {
                        my $c = modbus_connect($tgt_modbus_peer);
                        # magic sleep? mostly for Huawei SUN2000
                        if($sleepy){
                            logger::log_info("sleeping $sleepy seconds");
                            Time::HiRes::sleep($sleepy);
                        }
                        $c;
                    };

                    # loop multiple times, as a retry mechanism
                    my $redo_loop = 0;
                    while($redo_loop++ < $max_attempt_cnt){
                        last unless $::MB_LOOP;
                        wait_modbus_response($mb_conns->{$tgt_modbus_peer}, $unit_id, $register, $mb_msg, \&modbus_data_logger);
                    }
                };
                if(chomp(my $err = $@)){
                    delete $mb_conns->{$tgt_modbus_peer};
                    logger::log_error("problem reading $register from $tgt_modbus_peer, fd=".(defined $mb_fh?fileno($mb_fh):"<not connected>").":$err");
                    eval {
                        modbus_close($mb_fh);
                    };
                    if(chomp(my $cerr = $@)){
                        logger::log_error("error closing $tgt_modbus_peer, fd=".fileno($mb_fh).":$cerr");
                    }
                }
            }
            # close sockets/file descriptors, modbus doesn't like connections open
            modbus_close($_) for values %$mb_conns;
        }
    };
    if($@){
        logger::log_error($@);
    }
    return;
}

sub do_login {
    $0 = "modbus:login";
    my $uid = cfg("USER") // "installer";
    my $pwd = cfg("PASS") // "00000a";
    require Digest::SHA;

    # connect
    my $tgts = parse_cfg();
    my $tgt_modbus_peer = (keys %$tgts)[0];
    my $tgt_registers   = $tgts->{$tgt_modbus_peer};
    my $modbus_remote = modbus_connect($tgt_modbus_peer);

    # get challenge
    my $challenge;
    my $modbus_request_get_challenge = pack("CCS>", 0x41, 0x24, 0x01);
    wait_modbus_response($modbus_remote, 0, undef, $modbus_request_get_challenge, undef, sub {
        my ($f_code, $content) = @_;
        logger::log_info("response code: ".sprintf("%02x", $f_code).", value: 0x".to_hex($content));
        if($f_code == 0x41){
            my ($f_subcode, $c_1) = unpack("CC", substr($content, 0, 2, ''));
            if($f_subcode == 0x24 and $c_1 == 0x11){
                logger::log_info("got code ".sprintf("0x%02x/0x%02x/0x%02x", $f_code, $f_subcode, $c_1).", content: 0x".to_hex($content).", length: ".length($content));
                $challenge = $content;
            } else {
                logger::log_info("got code ".sprintf("0x%02x/0x%02x/0x%02x", $f_code, $f_subcode, $c_1));
            }
        } else {
            logger::log_error("not known function code: ".sprintf("0x%02x", $f_code));
        }
        return;
    });
    if(!defined $challenge){
        modbus_close($modbus_remote);
        return;
    }

    # hash challenge with secret/user and send response
    $challenge = substr($challenge, 0, 16);
    my $hashed_pwd = Digest::SHA::hmac_sha256($challenge, Digest::SHA::sha256($pwd));
    my $client_chg = do {open(my $rfh, '<', '/dev/urandom'); local $/ = \16; <$rfh>};
    my $modbus_request_login = pack("CCCa*Ca*Ca*", 0x41, 0x25,
        length($client_chg) + 1 + length($uid) + 1 + length($hashed_pwd),
        $client_chg,
        length($uid),
        $uid,
        length($hashed_pwd),
        $hashed_pwd);
    logger::log_info("got challenge ".to_hex($challenge).", length: ".length($challenge)." will log in with $uid/$pwd");
    logger::log_info("client challenge ".to_hex($client_chg).", hashed pass: ".to_hex($hashed_pwd));
    logger::log_info("will send response ".to_hex($modbus_request_login));
    Time::HiRes::usleep(500);
    my $challenge_response;
    wait_modbus_response($modbus_remote, 0, undef, $modbus_request_login, undef, sub {
        my ($f_code, $content) = @_;
        logger::log_info("response code: ".sprintf("%02x", $f_code).", value: ".to_hex($content));
        if($f_code == 0x41){
            my ($f_subcode, $r_sz) = unpack("CC", substr($content, 0, 2, ''));
            logger::log_info("got code ".sprintf("0x%02x/0x%02x size: %d", $f_code, $f_subcode, $r_sz).", content: ".to_hex($content).", length: ".length($content));
            if($f_subcode == 0x25){
                $challenge_response = $content;
            }
        } else {
            logger::log_error("not known function code: ".sprintf("0x%02x", $f_code));
        }
        return;
    });
    logger::log_info("got challenge response ".to_hex($challenge_response//""));

    my $msg = modbus_request_write_msg("SUN2000::Inverter::TimeZone", 60);
    wait_modbus_response($modbus_remote, 0, undef, $msg, undef, sub {
        logger::log_info("response code: 0x".sprintf("%02x", $_[0]).", value: 0x".to_hex($_[1]))
    });

    modbus_close($modbus_remote);
    return;
}

sub do_write {
    $0 = "modbus:write";
    my $tgts = parse_cfg();
    my $mb_conns = {};
    foreach my $r_entry (@$tgts){
        my ($tgt_modbus_peer, $unit_id, $register, $mb_msg, $value, $sleepy) = @$r_entry;
        next unless defined $value;
        my $mb_fh = $mb_conns->{$tgt_modbus_peer} //= do {
            my $c = modbus_connect($tgt_modbus_peer);
            if($sleepy){
                logger::log_info("sleeping $sleepy seconds");
                Time::HiRes::sleep($sleepy);
            }
            $c;
        };
        wait_modbus_response($mb_fh, $unit_id, $register, $mb_msg, undef, sub {
            logger::log_info("response code: 0x".sprintf("%02x", $_[0]).", value: 0x".to_hex($_[1]))
        });
    }
    modbus_close($_) for values %$mb_conns;
    return;
}

sub do_read {
    $0 = "modbus:read";
    return do_cli(@_);
}

sub to_hex {
    my $b = \$_[0];
    return join("",map {sprintf("%02x", ord $_)} split '', $$b//"");
}

sub wait_modbus_response {
    my ($modbus_remote, $unit_id, $remote_register, $msg, $data_logger, $process_response) = @_;
    return unless length($msg//"");

    # inbuffer/outbuffer is the message for modbus
    my ($inbuffer, $outbuffer) = ("", "");

    if(-S $modbus_remote or -p $modbus_remote or cfg("modbus_tcp", 0)){
        logger::log_debug("using socket modbus TCP header");
        # modbus TCP needs extra MBAP Header
        $::TRANSACTION_ID_COUNTER //= 0;
        # TCP ModBus needs the extra 6 byte header
        $msg  = pack("Ca".length($msg), $unit_id, $msg);
        $msg  = pack("S>S>S>a".length($msg), $::TRANSACTION_ID_COUNTER++, 0, length($msg), $msg);
        $outbuffer = $msg;
    } elsif(-c $modbus_remote){
        # RTU ModBus over rs232/rs485 UART
        logger::log_debug("using char uart modbus");
        $msg  = pack("Ca".length($msg), $unit_id, $msg);
        $outbuffer = $msg.modbus_crc($msg);
    }

    if(!length($outbuffer//"")){
        logger::log_debug("outbuffer empty");
        return;
    }
    logger::log_debug("OUTBUFFER HEX: ".to_hex($outbuffer));

    # prepare select mask, run select and verify result, on timeout
    # just exit the loop, buffer will be whatever is already there
    my $select_timeout = cfg("select_timeout", 5);
    my $do_timeout     = cfg("timeout", 15);
    my $max_nr_loops   = int($do_timeout/$select_timeout);
    $max_nr_loops      = $max_nr_loops <= 0?1:$max_nr_loops;
    my $fd = fileno($modbus_remote);
    my $rin = "";
    my $win = "";
    vec($rin, $fd, 1) = 1;
    my $nr_loops = 0;
  REDO_READ:
    die "Timeout reading data" if $nr_loops++ > $max_nr_loops;
  SELECT_LOOP:
    while($::MB_LOOP){
        local $! = 0;
        $win = length($outbuffer)?$rin:"";
        # stuff to do?
        my $r = select my $rout = $rin, $win, undef, $select_timeout;
        (!defined $r or $r == -1) and ($!{EINTR} or $!{EAGAIN} or die "select: $!\n");
        goto REDO_READ unless $r;
        logger::log_debug("FD[ROUT]: ".to_hex($rout).", FD[WOUT]: ".to_hex($rout));
        # write?
        if(length($outbuffer) and vec($win, $fd, 1)){
            my $buf = $outbuffer;
            $outbuffer = "";
            modbus_write($modbus_remote, $buf);
        }
        # read?
        if(vec($rout, $fd, 1)){
            while($::MB_LOOP){
                # read per 1 bytes, but until EAGAIN
                my $r = sysread($modbus_remote, my $response_data, 1);
                !defined $r and (($!{EAGAIN} and last SELECT_LOOP) or die "recv: $!\n");
                defined $r and !$r and die "EOF read on FD=$fd: $!\n";
                $inbuffer .= $response_data;
            }
        }
    }
    logger::log_debug("INBUFFER: ".to_hex($inbuffer));
    goto REDO_READ if length($inbuffer) < 8;

    # process MBAP modbus tcp frame header, on a copy of the buffer - this isn't much data
    my $buf = $inbuffer;
    my ($t_id, $_d, $r_sz) = unpack("S>S>S>", substr($buf, 0, 6, ''));
    goto REDO_READ if length($buf) < $r_sz;

    # process response
    $process_response //= sub {
        my ($f_code, $content) = @_;
        my $val;
        if($f_code == $MODBUS::READ_HOLDING_REGISTERS or $f_code == $MODBUS::READ_INPUT_REGISTERS){
            my ($nr_bytes, $data) = unpack('Ca*', $content);
            $val = modbus_response($remote_register, $nr_bytes, $data);
        } elsif($f_code == 0x06){
            my ($register_addr, $r_value, $r_crc) = unpack('S>S>S>', $content);
            $val = modbus_response($remote_register, 2, $r_value);
        } elsif($f_code == 0x83){
            my ($err_code) = unpack("C", $content);
            logger::log_error("problem read: ".sprintf("0x%02x", $err_code));
        } elsif($f_code == 0xc1){
            my ($err_code) = unpack("C", $content);
            logger::log_error("problem read: ".sprintf("0x%02x", $err_code));
        } elsif($f_code == 0x86){
            my ($err_code) = unpack("C", $content);
            logger::log_error("problem write: ".sprintf("0x%02x", $err_code));
        } elsif($f_code == 0x41){
            my ($f_subcode, $c_1) = unpack("CC", substr($content, 0, 2, ''));
            if($f_subcode == 0x05 and $c_1 == 0x11){
                logger::log_info("got code ".sprintf("0x%02x/0x%02x/0x%02x", $f_code, $f_subcode, $c_1).", content: ".to_hex($content));
            } else {
                logger::log_info("got code ".sprintf("0x%02x", $f_code));
            }
        } else {
            logger::log_error("not known function code: ".sprintf("0x%02x", $f_code));
        }
        &{$data_logger}($val) if defined $data_logger and defined $val;
        return
    };
    (undef, my $f) = unpack("CC", substr($buf, 0, 2, ''));
    &{$process_response}($f, $buf);
    return
}

sub parse_cfg {
    # loop over @ARGV, group per target host and loop over the registers to fetch, format:
    # [tcp://]<host[:port]>,<id:register>[=<newvalue>][,<id:register>[=<newvalue>]]
    # [rtu://]<uartdev>,<id:register>[=<newvalue>],[<id:register>[=<newvalue>]]
    my @tgts;
    foreach my $tgt_modbus_peer_entry (@ARGV){
        my $tgt = parse_register_cfg($tgt_modbus_peer_entry);
        push @tgts, $tgt if $tgt;
    }
    logger::log_info("will read ".join(",", map {join(";", map {$_//""} @$_[0..2,4])} @tgts)) if @tgts;
    print_usage("nothing to poll") unless @tgts;
    return \@tgts;
}

sub parse_register_cfg {
    my ($register_cfg) = @_;
    my ($tgt_modbus_peer, $regs) = split m/=/, $register_cfg, 2;
    my @registers_to_read = split m/,/, $regs;
    if(!@registers_to_read){
        my $modbus_peer = cfg("modbus_peer");
        if(!defined $modbus_peer){
            logger::log_error("no modbus target for $tgt_modbus_peer, use tcp:// or rtu://");
            next;
        }
        push @registers_to_read, $tgt_modbus_peer;
        $tgt_modbus_peer = $modbus_peer;
    }
    return unless @registers_to_read;
    return [$tgt_modbus_peer, map {parse_register($_)} @registers_to_read];
}

sub parse_register {
    my ($register_cfg) = @_;
    my ($unit_id, $register, $value) = $register_cfg =~ m/^(?:(\d+):)?(.*)(?:=([^,]+))?$/;
    my $mb_msg;
    if(defined $value){
        $value = hex($value);
        $mb_msg = modbus_request_write_msg($register, $value);
    } else {
        $mb_msg = modbus_request_read_msg($register);
    }
    my $sleep_after_connect = cfg("sleep_after_connect", $register_cfg =~ m/Huawei::/?2:undef);
    return $unit_id//0, $register, $mb_msg, $value, $sleep_after_connect;
}

sub modbus_crc {
    my ($buf) = @_;
    my $crc = 0xffff;
    my $i = 0;
    while($i < length($buf)){
        $crc ^= ord(substr($buf, $i++, 1));
        foreach my $j (0 .. 7){
            if(($crc & 0x0001) != 0){
                $crc >>= 1;
                $crc  ^= 0xa001;
            } else {
                $crc >>= 1;
            }
        }
    }
    return pack("S<", $crc);
}

sub load_mb_datatypes {
    my ($register_name) = @_;
    no warnings 'redefine';
    my $mb_product = "MODBUS::DATATYPE::".(${register_name} =~ s/^(.*)::.*?$/$1/gr);
    eval "local \@INC = (); require $mb_product";
    logger::log_debug("no such product class to load: $mb_product") if $@;
    return;
}

sub modbus_request_write_msg {
    my ($register_name, $value, $function_code) = @_;
    logger::log_debug("will write ".($value//to_hex($function_code))." to $register_name");
    no strict 'refs';
    load_mb_datatypes($register_name);
    my $register = (${"MODBUS::DATATYPE::${register_name}"})->[0] // die "No such register $register_name\n";
    my $fc       = (${"MODBUS::DATATYPE::${register_name}"})->[7];
    my $mr = pack("CS>S>", $fc // $function_code // $MODBUS::WRITE_HOLDING_REGISTERS, $register, $value);
    logger::log_debug("MODBUS WRITE REQUEST HEX [$register]: ".to_hex($mr));
    return $mr;
}

sub modbus_request_read_msg {
    my ($register_name, $function_code) = @_;
    no strict 'refs';
    load_mb_datatypes($register_name);
    my $register = (${"MODBUS::DATATYPE::${register_name}"})->[0] // die "No such register $register_name\n";
    my $cnt      = (${"MODBUS::DATATYPE::${register_name}"})->[1] // die "No such count for $register_name\n";
    my $fc       = (${"MODBUS::DATATYPE::${register_name}"})->[7];
    my $mr = pack("CS>S>", $fc // $function_code // $MODBUS::READ_HOLDING_REGISTERS, $register, $cnt);
    logger::log_debug("MODBUS READ REQUEST HEX [$register]: ".to_hex($mr));
    return $mr;
}

sub modbus_response {
    my ($register_name, $nr_bytes, $data) = @_;
    return unless length($data);
    no strict 'refs';
    load_mb_datatypes($register_name);
    my $fmt  = (${"MODBUS::DATATYPE::${register_name}"})->[2] // die "No unpack for $register_name\n";
    my $desc = (${"MODBUS::DATATYPE::${register_name}"})->[5] // $register_name;
    my $form = (${"MODBUS::DATATYPE::${register_name}"})->[6] // sub {$_[0]};
    my $unit = (${"MODBUS::DATATYPE::${register_name}"})->[4] // "";
    my $val  = unpack("$fmt$nr_bytes", $data);
    return unless defined $val;
    my $adjust = (${"MODBUS::DATATYPE::${register_name}"})->[3];
    logger::log_debug("return $val/$adjust for VAL: ".to_hex($data)." for $register_name");
    if(defined $adjust and defined $val and length($val) and $adjust != 0){
        my $new_val = eval "return $val/$adjust";
        if($@){
            logger::log_error($@);
        } else {
            $val = $new_val;
        }
    }
    $val = &{$form}($val);
    return lc($register_name =~ s/::/:/gr).($unit?"*$unit":"").",$val";
}

sub modbus_connect {
    my ($dest) = @_;

    # looks ok? support tcp and uart (file dev)
    ($dest //= "") =~ m/^(?:(tcp|rtu)?:\/\/)?(.*?)$/
        or die "Invalid connect target '$dest'\n";
    my $type = $1 // "";
    my $tgt  = $2 // "";

    if($type eq ""){
        # guess, based on regex, check for ipv4, ipv6, hostname or file and an
        # ip address can have an optional port
        $type = $tgt =~ m/^(?:(?:[0-9]{1,3}\.){3}[0-9]{1,3}|[0-9a-fA-F:]+|[\w\.\-]+|\/\w+)(?::\d+)?$/
            ?"tcp"
            :"rtu";
        $type = "rtu" if $tgt =~ m/^\/.*?/;
    }
    die "Invalid destination '$dest'\n"
        if $type !~ m/^(tcp|rtu)$/;

    # is a uart connection
    if($type eq "rtu"){
        return open_uart($tgt);
    }

    # is a tcp connection
    my ($dest_ip, $dest_port) = split m/:/, $tgt, 2;
    $dest_port ||= 6607;
    inet_aton($dest_ip)
        or die "Invalid destination '$dest' ($dest_ip, $dest_port)\n";
    logger::log_info("opening TCP $dest_ip:$dest_port");
    my $s;
    my $err;

    # missing in Socket
    my $SOL_TCP          =  6;
    my $TCP_KEEPIDLE     =  4; # Start keeplives after this period
    my $TCP_KEEPINTVL    =  5; # Interval between keepalives
    my $TCP_KEEPCNT      =  6; # Number of keepalives before death
    my $TCP_SYNCNT       =  7; # Number of SYN retransmits
    my $TCP_LINGER2      =  8; # Life time of orphaned FIN-WAIT-2 state
    my $TCP_DEFER_ACCEPT =  9; # Wake up listener only when data arrive
    my $TCP_WINDOW_CLAMP = 10; # Bound advertised window
    my $TCP_INFO         = 11; # Information about this connection.
    my $TCP_QUICKACK     = 12; # Bock/reenable quick ACKs.
    my $TCP_CONGESTION   = 13; # Congestion control algorithm.
    my $TCP_MD5SIG       = 14; # TCP MD5 Signature (RFC2385)

    eval {
      local $SIG{ALRM} = sub {die "ALARM: Timeout connecting to $dest\n"};
      eval {
        # create an empty stream tcp/ip socket
        socket($s, PF_INET, SOCK_STREAM, 0)
            // die "socket create problem: $!\n";

        # set specific socket options
        if(cfg("socketopts", 0)){
            if(my $sndtmo = cfg("sendtimeout", pack("qq", 30, 0))){
                setsockopt($s, SOL_SOCKET, SO_SNDTIMEO, $sndtmo)
                    // logger::log_error("problem setting SO_SNDTIMEO=$sndtmo: $!");
            }
            if(my $sndbuf = cfg("sendbuffer", 1_048_576)){
                setsockopt($s, SOL_SOCKET, SO_SNDBUF, $sndbuf)
                    // logger::log_error("problem setting SO_SNDBUF=$sndbuf: $!");
            }
            if(my $rcvbuf = cfg("receivebuffer", 1_048_576)){
                setsockopt($s, SOL_SOCKET, SO_RCVBUF, $rcvbuf)
                    // logger::log_error("problem setting SO_RCVBUF=$rcvbuf: $!");
            }
            if(my $ka = cfg("so_keepalive", 1)){
                # see https://man7.org/linux/man-pages/man7/tcp.7.html
                my ($kt, $kn, $ki);
                $kt //= cfg("tcp_keepalive_time",   60);
                $kn //= cfg("tcp_keepalive_probes",  5);
                $ki //= cfg("tcp_keepalive_intvl",  12);
                setsockopt($s, SOL_SOCKET, SO_KEEPALIVE, $ka+0)
                    // logger::log_error("problem setting SO_KEEPALIVE=$ka: $!");
                setsockopt($s, $SOL_TCP, $TCP_KEEPIDLE, $kt+0)
                    // logger::log_error("problem setting $TCP_KEEPIDLE=$kt: $!");
                setsockopt($s, $SOL_TCP, $TCP_KEEPCNT, $kn+0)
                    // logger::log_error("problem setting $TCP_KEEPCNT=$kn: $!");
                setsockopt($s, $SOL_TCP, $TCP_KEEPINTVL, $ki+0)
                    // logger::log_error("problem setting $TCP_KEEPINTVL=$ki: $!");
            }
            if(my $nd = cfg("tcp_nodelay", 1)){
                # see https://man7.org/linux/man-pages/man7/tcp.7.html
                setsockopt($s, $SOL_TCP, TCP_NODELAY, $nd+0)
                    // logger::log_error("problem setting TCP_NODELAY=$nd: $!");
            }
            if(my $co = cfg("tcp_cork", 1)){
                # see https://man7.org/linux/man-pages/man7/tcp.7.html
                setsockopt($s, $SOL_TCP, TCP_CORK, $co+0)
                    // logger::log_error("problem setting TCP_CORK=$co: $!");
            }
        }

        # now connect
        alarm cfg("connect_timeout", 10);
        connect($s, pack_sockaddr_in($dest_port, inet_aton($dest_ip)))
            or do {
                alarm 0;
                die "connect failed to $dest_ip:$dest_port: $!\n";
            };
        alarm 0;
        binmode($s);
        my $s_flags = fcntl($s, F_GETFL, 0)
            or die "Can't get flags for the socket: $!\n";
        fcntl($s, F_SETFL, $s_flags | O_NONBLOCK)
            or die "Can't set flags for the socket: $!\n";
      };
      alarm 0;
      $err = $@ if $@;
    };
    if(chomp(my $cerr = $@)){
        # this only happens on a possible late alarm 0 on error
        logger::log_error($cerr);
    }
    if($err){
        close($s) if defined $s;
        $! = 0;
        die $err;
    }
    return $s;
}

sub modbus_write {
    my ($fd, $msg) = @_;
    REDO_SEND:
    my $w = syswrite($fd, $msg);
    !defined $w and (($!{EAGAIN} and goto REDO_SEND) and die "sendto: $!\n");
    defined $w and $w != length($msg)
        and die "not enough bytes sent: $w, wanted ".length($msg)."\n";
    return;
}

sub modbus_close {
    my ($fh) = @_;
    return unless defined $fh;
    close($fh)
        or logger::log_error("close: $!");
    return;
}

sub open_uart {
    my ($dev) = @_;
    logger::log_info("will use UART $dev");
    $dev ||= "/dev/ttyUSB0";
    logger::log_error("no such device $dev")
        unless -c $dev;
    logger::log_info("opening UART $dev");
    sysopen(my $com, $dev, O_RDWR)
         or die "Cannot open serial port $dev: $!\n";
    # TCGETS
    use constant TCGETS  => 0x5401;
    use constant TCSETSW => 0x5403;
    ioctl($com, TCGETS, my $tty_flags = "") == 0
        or die "ioctl failed: $!\n";
    logger::log_debug("UART flags GOT: ".to_hex($tty_flags));
    # TCSETSW 115200, 8N1
    use constant ICRNL     => 0x00000040;
    use constant INLCR     => 0x00000040;
    use constant IXON      => 0x00000200;
    use constant IXOFF     => 0x00000400;
    use constant OCRNL     => 0x00000010;
    use constant ONLCR     => 0x00000002;
    use constant PARENB    => 0x00001000;
    use constant CSTOPB    => 0x00002000;
    use constant NOFLSH    => 0x80000000;
    use constant ISIG      => 0x00000080;
    use constant ICANON    => 0x00000002;
    use constant ECHO      => 0x00000008;
    use constant ECHOE     => 0x00000002;
    use constant ECHOK     => 0x00000004;
    use constant ECHOKE    => 0x00000008;
    use constant ECHOCTL   => 0x00000040;
    use constant OPOST     => 0x00000001;
    use constant CSIZE     => 0x00000060;
    use constant CS8       => 0x00000060;
    use constant CBAUD     => 0x00010017;
    use constant CBAUDEX   => 0x00010000;
    use constant B115200   => 0x00010002;
    use constant CRTSCTS   => 0x00030000;
    my $sgttyb_t = "llll";
    my $sgttyb = pack($sgttyb_t,
        ICRNL|INLCR & (~IXON|~IXOFF),
        OCRNL|ONLCR & (~OPOST),
        ((CBAUD|CBAUDEX) & B115200)|(CSIZE & CS8)|PARENB & (~CSTOPB|~CRTSCTS),
        0 & (~NOFLSH|~ISIG|~ICANON|~ECHO|~ECHOE|~ECHOK|~ECHOKE|~ECHOCTL), 0);
    logger::log_debug("UART flags SET: ".to_hex($sgttyb));
    ioctl($com, TCSETSW, $sgttyb) == 0
        or die "ioctl failed: $!\n";
    binmode($com);
    fcntl($com, F_SETFL, O_RDWR|O_NONBLOCK)
        // die "Failed non-blocking set on $com: $!\n";
    return $com;
}

sub close_uart {
    my ($comdev) = @_;
    return unless defined $comdev and defined $$comdev;
    close($$comdev)
        or die "Error closing UART FH=".fileno($$comdev).": $!\n";
    $$comdev = undef;
    return;
}

sub write_uart {
    my ($comdev, @data) = @_;
    print {$comdev} join("", @data)."\n";
    return;
}

our $logger_sub;
sub modbus_data_logger {
    my ($val_str) = @_;
    $logger_sub //= do {
        # prefix
        my $key_prefix = cfg("key_prefix");
        my $prefix = lc(length($key_prefix//"") ? "$key_prefix:": "");

        my $l_sub;
        my $sink = cfg("sink");
        if($sink =~ m/^(udp|unix):\/\/(.*?)(?::(\d+))?$/i){
            my ($s, $t_addr);
            logger::log_info("will log to $sink");
            my $tgt_sink_scheme   = $1;
            my $tgt_udp_sink_host = $2;
            my $tgt_udp_sink_port = $3;
            if($tgt_sink_scheme eq 'udp'){
                my $h_addr = inet_aton($tgt_udp_sink_host)
                    // die "unknown host: $tgt_udp_sink_host";
                $t_addr = sockaddr_in($tgt_udp_sink_port, $h_addr);
                my $proto  = getprotobyname("udp");
                socket($s, PF_INET, SOCK_DGRAM, $proto)
                    // die "socket create problem: $!\n";
            } elsif($tgt_sink_scheme eq 'unix'){
                $t_addr = pack_sockaddr_un($tgt_udp_sink_host);
                socket($s, PF_UNIX, SOCK_DGRAM, 0)
                    // die "socket create problem: $!\n";
            }
            fcntl($s, F_SETFL, O_RDWR|O_NONBLOCK)
                // die "socket non-blocking set problem: $!\n";
            binmode($s)
                // die "binmode problem: $!\n";
            $l_sub = sub {
                my ($v_str) = @_;
                return unless defined $v_str;

                # get current time
                my ($tm, $usec) = Time::HiRes::gettimeofday();
                my $ts = POSIX::strftime("%F %H:%M:%S.".sprintf("%06d", $usec), localtime($tm));

                # and make log line
                my $data = "$ts,$prefix$v_str\n";

                if(length($data) <= 1024){
                    send($s, $data, 0, $t_addr)
                        // logger::log_error("problem sending data to $sink: $!");
                } else {
                    # split, recombine and send
                    my @m = split m/\n/, $data;
                    my $buf = shift @m;
                    $buf .= "\n";
                    while(@m){
                        my $next_m = shift @m;
                        $next_m .= "\n";
                        if(length($buf) + length($next_m) <= 1024){
                            $buf .= $next_m;
                            next;
                        }
                        send($s, $buf, 0, $t_addr)
                            // logger::log_error("problem sending data to $sink: $!");
                        $buf = $next_m;
                    }
                }
                return;
            };
        } elsif($sink !~ m/^tcp:\/\//i){
            open(my $ofh, $sink)
                or die "Error opening $sink: $!\n";
            fcntl($ofh, F_SETFL, O_RDWR|O_NONBLOCK)
                // die "socket non-blocking set problem: $!\n";
            binmode($ofh)
                // die "binmode problem: $!\n";
            $l_sub = sub {
                my ($v_str) = @_;
                return unless defined $v_str;

                # get current time
                my ($tm, $usec) = Time::HiRes::gettimeofday();
                my $ts = POSIX::strftime("%F %H:%M:%S.".sprintf("%06d", $usec), localtime($tm));

                # and make log line and flush
                print {$ofh} "$ts,$prefix$v_str\n";
                my $old_fh = select $ofh; $|=1; select $old_fh;
            };
        } else {
            die "$sink is invalid\n";
        }
        $l_sub;
    };
    return &{$logger_sub}($val_str);
};

sub print_usage {
    my ($err) = @_;
    print "usage: $0 [tcp://]<uartdev|host[:port(default:6607)]> <register> [<register>]\n";
    print "\n$err\n\n" if length($err//"");
    exit 1;
}

our $attempted_load;
sub cfg {
    my ($key, $default_v) = @_;
    if(!$attempted_load){
        $attempted_load = 1;
        eval {require utils::cfg};
    }
    my $orig_cfg = \&utils::cfg;
    no warnings 'redefine';
    *utils::cfg = sub {
        my ($key, $default_v) = @_;
        return $ENV{uc($key)} // $default_v;
    };
    return utils::cfg($key, $default_v);
}

package logger;

use strict; use warnings;

no warnings 'redefine';
no warnings 'once';

BEGIN {
    eval {require utils::logger};
    if($@){
        *log_debug = \&just_print;
        *log_info  = \&log_print;
        *log_error = \&log_print;
        *log_fatal = \&log_print;
    } else {
        *log_debug = \&logger::log_debug;
        *log_info  = \&logger::log_info;
        *log_error = \&logger::log_error;
        *log_fatal = \&logger::log_fatal;
    }
}

sub just_print {
    my (@msg) = @_;
    return if !$ENV{DEBUG};
    return log_print(@msg);
}

sub log_print {
    my (@msg) = @_;
    $::LOG_PREFIX //= "";
    require Time::HiRes;
    require POSIX;
    my ($tm, $usec) = Time::HiRes::gettimeofday();
    $usec = sprintf("%06d", $usec);
    my @tm = localtime($tm);
    my $msg = join("\n", map {POSIX::strftime("%H:%M:%S.$usec", @tm)." [$$]: $::LOG_PREFIX$_"} map {split m/\n/, $_//""} @msg);
    print STDERR "$msg\n";
    return;
}

package main;

BEGIN {

package MODBUS;

our $READ_HOLDING_REGISTERS  = 0x03;
our $READ_INPUT_REGISTERS    = 0x04;
our $WRITE_HOLDING_REGISTERS = 0x06;
our $WRITE_PRIVATE_REGISTERS = 0x41;

package MODBUS::DATATYPE::DUCO::DUCOBoxSilent;

our $Nodes0       = [0x0000, 1, "s>", 1, undef, "Nodes0", sub {$_[0]}, $MODBUS::READ_INPUT_REGISTERS];
our $BoxModelType = [0x0100, 1, "s>", 1, undef, "BoxModelType", sub {$_[0]}, $MODBUS::READ_INPUT_REGISTERS];

package MODBUS::DATATYPE::Huawei::SUN2000::SDongleA;

our $OSVersion       = ["30000", 15, "a*"];
our $ProtocolVersion = ["30068",  1, "a" , 2];
our $SN              = ["30015", 10, "a*"];
our $Type            = ["37410",  2, "S>"];

package MODBUS::DATATYPE::Huawei::SUN2000::Inverter;

# see https://www-file.huawei.com/~/media/CORPORATE/PDF/FusionSolar/HUAWEI_SUN2000_245KTL28KTL_MODBUS_Interface_Definitions_20150715_JP.pdf
# see https://photomate.zendesk.com/hc/en-gb/article_attachments/20147524663581
# https://www.debacher.de/wiki/Sun2000_Modbus_Register

sub _device_status {
    my ($val) = @_;
    my $st = {
        0     => " [Standby: initializing]",
        1     => " [Standby: detecting insulation resistance]",
        2     => " [Standby: detecting irradiation]",
        3     => " [Standby: grid detecting]",
        256   => " [Starting]",
        512   => " [On-grid: running]",
        513   => " [Grid connection: power limited]",
        514   => " [Grid connection: self-derating]",
        515   => " [Off-grid Running]",
        768   => " [Shutdown: fault]",
        769   => " [Shutdown: command]",
        770   => " [Shutdown: OVGR]",
        771   => " [Shutdown: communication disconnected]",
        772   => " [Shutdown: power limited]",
        773   => " [Shutdown: manual startup required]",
        774   => " [Shutdown: DC switches disconnected]",
        775   => " [Shutdown: rapid cutoff]",
        776   => " [Shutdown: input underpower]",
        1025  => " [Grid scheduling: cosΦ-P curve]",
        1026  => " [Grid scheduling: Q-U curve]",
        1027  => " [Grid scheduling: PF- U curve]",
        1028  => " [Grid scheduling: dry contact]",
        1029  => " [Grid scheduling: Q-P curve]",
        1280  => " [Spot- check ready]",
        1281  => " [Spot- checking]",
        1536  => " [Inspecting]",
        1792  => " [AFCI self check]",
        2048  => " [I-V scanning]",
        2304  => " [DC input detection]",
        2560  => " [Running: off- grid charging]",
        40960 => " [Standby: no irradiation ]",
    };
    return $val.($st->{$val} // " [unknown]");
}

sub _date_fmt {
    my ($val) = @_;
    return POSIX::strftime("%FT%T", gmtime($val));
}

sub _bitstring_fmt {
    my ($val) = @_;
    return reverse unpack "b*", reverse $val;
}

our $Model                                   = ["30000", 15, "a*"];
our $SN                                      = ["30015", 10, "a*"];
our $PN                                      = ["30025", 10, "a*"];
our $ModelID                                 = ["30070", 1, "S>", 1];
our $NumberOfPVStrings                       = ["30071", 1, "S>", 1];
our $NumberOfMPPTrackers                     = ["30072", 1, "S>", 1];
our $RatedPower                              = ["30073", 2, "L>", 1, "W"];
our $MaximumActivePower                      = ["30075", 2, "L>", 1, "W"];
our $MaximumApparentPower                    = ["30077", 2, "L>", 1000, "kVA"];
our $MaximumReactivePowerFedToTheGrid        = ["30079", 2, "l>", 1000, "kvar"];
our $MaximumReactivePowerAbsorbedFromTheGrid = ["30081", 2, "l>", 1000, "kvar"];
our $State1                                  = ["32000", 1, "a*", undef, undef, undef, \&_bitstring_fmt];
our $State2                                  = ["32002", 1, "a*", undef, undef, undef, \&_bitstring_fmt];
our $State3                                  = ["32003", 2, "a*", undef, undef, undef, \&_bitstring_fmt];
our $Alarm1                                  = ["32008", 1, "a*", undef, undef, undef, \&_bitstring_fmt];
our $Alarm2                                  = ["32009", 1, "a*", undef, undef, undef, \&_bitstring_fmt];
our $Alarm3                                  = ["32010", 1, "a*", undef, undef, undef, \&_bitstring_fmt];
our $PV1Voltage                              = ["32016", 1, "s>", 10, "V"];
our $PV1Current                              = ["32017", 1, "s>", 100, "A"];
our $PV2Voltage                              = ["32018", 1, "s>", 10, "V"];
our $PV2Current                              = ["32019", 1, "s>", 100, "A"];
our $PV3Voltage                              = ["32020", 1, "s>", 10, "V"];
our $PV3Current                              = ["32021", 1, "s>", 100, "A"];
our $PV4Voltage                              = ["32022", 1, "s>", 10, "V"];
our $PV4Current                              = ["32023", 1, "s>", 100, "A"];
our $PV5Voltage                              = ["32024", 1, "s>", 10, "V"];
our $PV5Current                              = ["32025", 1, "s>", 100, "A"];
our $PV6Voltage                              = ["32026", 1, "s>", 10, "V"];
our $PV6Current                              = ["32027", 1, "s>", 100, "A"];
our $PV7Voltage                              = ["32028", 1, "s>", 10, "V"];
our $PV7Current                              = ["32029", 1, "s>", 100, "A"];
our $PV8Voltage                              = ["32030", 1, "s>", 10, "V"];
our $PV8Current                              = ["32031", 1, "s>", 100, "A"];
our $PV9Voltage                              = ["32032", 1, "s>", 10, "V"];
our $PV9Current                              = ["32033", 1, "s>", 100, "A"];
our $PV10Voltage                             = ["32034", 1, "s>", 10, "V"];
our $PV10Current                             = ["32035", 1, "s>", 100, "A"];
our $PV11Voltage                             = ["32036", 1, "s>", 10, "V"];
our $PV11Current                             = ["32037", 1, "s>", 100, "A"];
our $PV12Voltage                             = ["32038", 1, "s>", 10, "V"];
our $PV12Current                             = ["32039", 1, "s>", 100, "A"];
our $PV13Voltage                             = ["32040", 1, "s>", 10, "V"];
our $PV13Current                             = ["32041", 1, "s>", 100, "A"];
our $PV14Voltage                             = ["32042", 1, "s>", 10, "V"];
our $PV14Current                             = ["32043", 1, "s>", 100, "A"];
our $PV15Voltage                             = ["32044", 1, "s>", 10, "V"];
our $PV15Current                             = ["32045", 1, "s>", 100, "A"];
our $PV16Voltage                             = ["32046", 1, "s>", 10, "V"];
our $PV16Current                             = ["32047", 1, "s>", 100, "A"];
our $PV17Voltage                             = ["32048", 1, "s>", 10, "V"];
our $PV17Current                             = ["32049", 1, "s>", 100, "A"];
our $PV18Voltage                             = ["32050", 1, "s>", 10, "V"];
our $PV18Current                             = ["32051", 1, "s>", 100, "A"];
our $PV19Voltage                             = ["32052", 1, "s>", 10, "V"];
our $PV19Current                             = ["32053", 1, "s>", 100, "A"];
our $PV20Voltage                             = ["32054", 1, "s>", 10, "V"];
our $PV20Current                             = ["32055", 1, "s>", 100, "A"];
our $PV21Voltage                             = ["32056", 1, "s>", 10, "V"];
our $PV21Current                             = ["32057", 1, "s>", 100, "A"];
our $PV22Voltage                             = ["32058", 1, "s>", 10, "V"];
our $PV22Current                             = ["32059", 1, "s>", 100, "A"];
our $PV23Voltage                             = ["32060", 1, "s>", 10, "V"];
our $PV23Current                             = ["32061", 1, "s>", 100, "A"];
our $PV24Voltage                             = ["32062", 1, "s>", 10, "V"];
our $PV24Current                             = ["32063", 1, "s>", 100, "A"];
our $InputPower                              = ["32064", 2, "l>", 1, "W"];
our $LineVoltageBetweenPhasesAAndB           = ["32066", 1, "S>", 10, "V"];
our $LineVoltageBetweenPhasesBAndC           = ["32067", 1, "S>", 10, "V"];
our $LineVoltageBetweenPhasesCAndA           = ["32068", 1, "S>", 10, "V"];
our $PhaseAVoltage                           = ["32069", 1, "S>", 10, "V"];
our $PhaseBVoltage                           = ["32070", 1, "S>", 10, "V"];
our $PhaseCVoltage                           = ["32071", 1, "S>", 10, "V"];
our $PhaseACurrent                           = ["32072", 2, "l>", 1000, "A"];
our $PhaseBCurrent                           = ["32074", 2, "l>", 1000, "A"];
our $PhaseCCurrent                           = ["32076", 2, "l>", 1000, "A"];
our $PeakActivePowerOfCurrentDay             = ["32078", 2, "l>", 1, "W"];
our $ActivePower                             = ["32080", 2, "l>", 1, "W"];
our $ReactivePower                           = ["32082", 2, "l>", 1000, "kvar"];
our $PowerFactor                             = ["32084", 1, "s>", 1000];
our $GridFrequency                           = ["32085", 1, "S>", 100, "Hz"];
our $Efficiency                              = ["32086", 1, "S>", 100, "%"];
our $InternalTemperature                     = ["32087", 1, "s>", 10, "°C"];
our $InsulationResistance                    = ["32088", 1, "S>", 1000, "MOhm"];
our $DeviceStatus                            = ["32089", 1, "S>", 1, undef, undef, \&_device_status];
our $FaultCode                               = ["32090", 1, "S>", 1];
our $StartupTime                             = ["32091", 2, "L>", 1, undef, undef, \&_date_fmt];
our $ShutdownTime                            = ["32093", 2, "L>", 1, undef, undef, \&_date_fmt];
our $AccumulatedEnergyYield                  = ["32106", 2, "L>", 100, "kWh"];
our $DailyEnergyYield                        = ["32114", 2, "L>", 100, "kWh"];
our $ActiveAdjustmentMode                    = ["35300", 1, "S>", 1];
our $ActiveAdjustmentValue                   = ["35302", 2, "L>", 1];
our $ActiveAdjustmentCommand                 = ["35303", 1, "S>", 1];
our $ReactiveAdjustmentMode                  = ["35304", 1, "S>", 1];
our $ReactiveAdjustmentValue                 = ["35305", 2, "L>", 1];
our $ReactiveAdjustmentCommand               = ["35307", 1, "S>", 1];
our $PowerMeterCollectionActivePower         = ["37113", 2, "l>", 1, "W"];
our $TotalNumberOfOptimizers                 = ["37200", 1, "S>", 1];
our $NumberOfOnlineOptimizers                = ["37201", 1, "S>", 1];
our $FeatureData                             = ["37202", 1, "S>", 1];
our $SystemTime                              = ["40000", 2, "L>", 1, undef, undef, \&_date_fmt];
our $QUCharacteristicCurveMode               = ["40037", 1, "S>", 1];
our $QUDispatchTriggerPower                  = ["40038", 1, "S>", 1, "%"];
our $FixedActivePowerDeratedInKW             = ["40120", 1, "S>", 10, "kW"];
our $ReactivePowerCompensationInPF           = ["40122", 1, "s>", 1000];
our $ReactivePowerCompensationQS             = ["40123", 1, "s>", 1000];
our $ActivePowerPercentageDerating           = ["40125", 1, "S>", 10, "%"];
our $FixedActivePowerDeratedInW              = ["40126", 2, "L>", 1, "W"];
our $ReactivePowerCompensationAtNight        = ["40129", 2, "l>", 1000, "kvar"];
our $CosPhiPPnCharacteristicCurve            = ["40133", 21, "a*"];
our $QUCharacteristicCurve                   = ["40154", 21, "a*"];
our $PFUCharacteristicCurve                  = ["40175", 21, "a*"];
our $ReactivePowerAdjustmentTime             = ["40196", 1, "S>", 1, "s"];
our $QUPowerPercentageToExitScheduling       = ["40198", 1, "S>", 1, "%"];
#our $Startup                                 = ["40200", 1, "S>", 1]; # disabled because not readable
#our $Shutdown                                = ["40201", 1, "S>", 1]; # disabled because not readable
our $GridCode                                = ["42000", 1, "S>", 1];
our $ReactivePowerChangeGradient             = ["42015", 2, "L>", 1000, "%/s"];
our $ActivePowerChangeGradient               = ["42017", 2, "L>", 1000, "%/s"];
our $ScheduleInstructionValidDuration        = ["42019", 2, "L>", 1, "s"];
our $TimeZone                                = ["43006", 1, "s>", 1, "min"];


package MODBUS::DATATYPE::Huawei::SUN2000::Meter;

our $MeterType                 = ["37125", 1, "S>", 1];
our $MeterStatus               = ["37100", 1, "S>", 1];
our $MeterModelDetectionResult = ["37138", 1, "S>", 1];
our $APhaseVoltage             = ["37101", 2, "l>", 10, "V"];
our $BPhaseVoltage             = ["37103", 2, "l>", 10, "V"];
our $CPhaseVoltage             = ["37105", 2, "l>", 10, "V"];
our $APhaseCurrent             = ["37107", 2, "l>", 100, "A"];
our $BPhaseCurrent             = ["37109", 2, "l>", 100, "A"];
our $CPhaseCurrent             = ["37111", 2, "l>", 100, "A"];
our $ActivePower               = ["37113", 2, "l>", 1, "W"];
our $ReactivePower             = ["37115", 2, "l>", 1, "var"];
our $PowerFactor               = ["37117", 1, "s>", 1000];
our $GridFrequency             = ["37118", 1, "s>", 100, "Hz"];
our $PositiveActiveElectricity = ["37119", 2, "l>", 100, "kWh"];
our $ReverseActivePower        = ["37121", 2, "l>", 100, "kWh"];
our $AccumulatedReactivePower  = ["37123", 2, "l>", 100, "kvar"];
our $ABLineVoltage             = ["37126", 2, "l>", 10, "V"];
our $BCLineVoltage             = ["37128", 2, "l>", 10, "V"];
our $CALineVoltage             = ["37130", 2, "l>", 10, "V"];
our $APhaseActivePower         = ["37132", 2, "l>", 1, "W"];
our $BPhaseActivePower         = ["37134", 2, "l>", 1, "W"];
our $CPhaseActivePower         = ["37136", 2, "l>", 1, "W"];

package MODBUS::DATATYPE::Huawei::LUNA2000::Battery;

sub _running_status {
    my ($val) = @_;
    my $rs_bat = {
        0 => " [offline]",
        1 => " [stand by]",
        2 => " [running]",
        3 => " [fault]",
        4 => " [sleep mode]",
    };
    return $val.($rs_bat->{$val}//" [unknown]");
}

sub _working_mode {
    my ($val) = @_;
    my $wm_bat = {
        0 => " [Adaptive(Fixed charge/ discharge / Maximise self consumption)]",
        1 => " [Fixed charge/ discharge]",
        2 => " [Maximise selfconsumption]",
        3 => " [Time Of Use(LG)]",
        4 => " [Fully fed to grid]",
        5 => " [Time Of Use(Luna)]",
    };
    return $val.($wm_bat->{$val}//" [unknown]");
}

sub _unit_working_mode {
    my ($val) = @_;
    my $wm_bat = {
        0  => " [none]",
        1  => " [Forcible charge/discharge]",
        2  => " [Time of Use(LG)]",
        3  => " [Fixed charge/discharge]",
        4  => " [Maximise selfconsumption]",
        5  => " [Fully fed to grid]",
        6  => " [Time of Use(LUNA2000)]",
        7  => " [remote schedulingmaximum self-use]",
        8  => " [remote scheduling - full Internet access]",
        9  => " [remote scheduling - TOU]",
        10 => " [AI energy management and scheduling]",
    };
    return $val.($wm_bat->{$val}//" [unknown]");
}

my $ap_bat = {
    0 => " [Unlimited (default)]",
    1 => " [Active scheduling]",
    5 => " [Zero power grid connection]",
    6 => " [Power - limited grid connection (kW)]",
    7 => " [Power - limited grid connection (%)]",
};

sub _enable_fmt {
    my ($val) = @_;
    return "$val [".($val eq 1?"enable":$val eq 0?"disable":"unknown")."]";
}

# Overall
our $RunningStatus                          = ["37762", 1, "S>", 1, undef, undef, \&_running_status];
our $WorkingModeSettings                    = ["47086", 1, "S>", 1, undef, undef, \&_working_mode];
our $BusVoltage                             = ["37763", 1, "S>", 10, "V"];
our $BusCurrent                             = ["37764", 1, "s>", 10, "A"];
our $ChargeDischargePower                   = ["37765", 2, "l>", 1, "W"];
our $MaximumChargePower                     = ["37046", 2, "l>", 1, "W"];
our $MaximumDischargePower                  = ["37048", 2, "l>", 1, "W"];
our $RatedCapacity                          = ["37758", 2, "l>", 1, "Wh"];
our $SOC                                    = ["37760", 1, "S>", 10, "%"];
our $BackupPowerSOC                         = ["47102", 1, "S>", 10, "%"];
our $TotalCharge                            = ["37780", 2, "l>", 100, "kWh"];
our $TotalDischarge                         = ["37782", 2, "l>", 100, "kWh"];
our $CurrentDayChargeCapacity               = ["37784", 2, "l>", 100, "kWh"];
our $CurrentDayDischargeCapacity            = ["37786", 2, "l>", 100, "kWh"];
our $TimeOfUseElectricityPricePeriods       = ["47028", 41, "a*"];
our $MaximumChargingPower                   = ["47075", 2, "l>", 1, "W"];
our $MaximumDischargingPower                = ["47077", 2, "l>", 1, "W"];
our $ChargingCutoffCapacity                 = ["47081", 1, "S>", 10, "%"];
our $DischargeCutoffCapacity                = ["47082", 1, "S>", 10, "%"];
our $ForcedChargingAndDischargingPeriod     = ["47083", 1, "S>", 1, "minutes"];
our $ChargeFromGridFunction                 = ["47087", 1, "S>", 1, undef, undef, \&_enable_fmt];
our $GridChargeCutoffSOC                    = ["47088", 1, "S>", 10, "%"];
our $ForcibleChargeDischarge                = ["47100", 1, "S>", 1, undef, undef, sub {"$_[0] [".($_[0] eq 0?"stop":$_[0] eq 1?"charge":$_[0] eq 2?"discharge":"unknown")."]"}];
our $FixedChargingAndDischargingPeriods     = ["47200", 41, "a*"];
our $PowerOfChargeFromGrid                  = ["47242", 2, "l>", 0.1, "W"];
our $MaximumPowerOfChargeFromGrid           = ["47244", 2, "l>", 0.1, "W"];
our $ForcibleChargeDischargeSettingMode     = ["47246", 1, "S>", 1, undef, undef, \&_enable_fmt];
our $ForcibleChargePower                    = ["47247", 2, "l>", 0.1, "W"];
our $ForcibleDischargePower                 = ["47249", 2, "l>", 0.1, "W"];
our $TimeOfUseChargingAndDischargingPeriods = ["47255", 43, "a*"];
our $ExcessPVEnergyUseInTOU                 = ["47299", 1, "S>", 1];
our $ActivePowerControlMode                 = ["47415", 1, "S>", 1, undef, undef, sub {"$_[0]$ap_bat->{$_[0]}"}];
our $MaximumFeedGridPowerInKW               = ["47416", 2, "l>", 1000, "kW"];
our $MaximumFeedGridPowerInPercentage       = ["47418", 1, "s>", 10, "%"];
our $MaximumChargeFromGridPower             = ["47590", 2, "l>", 0.1, "W"];
our $SwitchToOffGrid                        = ["47604", 1, "S>", 1, undef, undef, \&_enable_fmt];
our $VoltageInIndependentOperation          = ["47605", 1, "S>", 1];

# Unit 1
our $Unit1ProductModel                      = ["47000", 1, "S>", 1];
our $Unit1SN                                = ["37052", 10, "a*"];
our $Unit1No                                = ["47107", 1, "S>", 1];
our $Unit1SoftwareVersion                   = ["37814", 15, "a*"];
our $Unit1DCDCVersion                       = ["37026", 10, "a*"];
our $Unit1BMSVersion                        = ["37036", 10, "a*"];
our $Unit1RunningStatus                     = ["37000", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit1WorkingMode                       = ["37006", 1, "S>", 1, undef, undef, \&_unit_working_mode];
our $Unit1BusVoltage                        = ["37003", 1, "S>", 10, "V"];
our $Unit1BusCurrent                        = ["37021", 1, "s>", 10, "A"];
our $Unit1BatterySOC                        = ["37004", 1, "S>", 10, "%"];
our $Unit1ChargeAndDischargePower           = ["37001", 2, "l>", 1, "W"];
our $Unit1RemainingChargeDischargeTime      = ["37025", 1, "S>", 1, "minutes"];
our $Unit1RatedChargePower                  = ["37007", 2, "l>", 1, "W"];
our $Unit1RatedDischargePower               = ["37009", 2, "l>", 1, "W"];
our $Unit1CurrentDayChargeCapacity          = ["37015", 2, "l>", 100, "kWh"];
our $Unit1CurrentDayDischargeCapacity       = ["37017", 2, "l>", 100, "kWh"];
our $Unit1TotalCharge                       = ["37066", 2, "l>", 100, "kWh"];
our $Unit1TotalDischarge                    = ["37068", 2, "l>", 100, "kWh"];
our $Unit1BatteryTemperature                = ["37022", 1, "s>", 10, "°C"];
our $Unit1FaultID                           = ["37014", 1, "S>", 1];

# Unit 2
our $Unit2ProductModel                      = ["47089", 1, "S>", 1];
our $Unit2SN                                = ["37700", 10, "a*"];
our $Unit2No                                = ["47108", 1, "S>", 1];
our $Unit2SoftwareVersion                   = ["37799", 15, "a*"];
our $Unit2RunningStatus                     = ["37741", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit2BusVoltage                        = ["37750", 1, "S>", 10, "V"];
our $Unit2BusCurrent                        = ["37751", 1, "s>", 10, "A"];
our $Unit2BatterySOC                        = ["37738", 1, "S>", 10, "%"];
our $Unit2ChargeAndDischargePower           = ["37743", 2, "l>", 1, "W"];
our $Unit2CurrentDayChargeCapacity          = ["37746", 2, "l>", 100, "kWh"];
our $Unit2CurrentDayDischargeCapacity       = ["37748", 2, "l>", 100, "kWh"];
our $Unit2TotalCharge                       = ["37753", 2, "l>", 100, "kWh"];
our $Unit2TotalDischarge                    = ["37755", 2, "l>", 100, "kWh"];
our $Unit2BatteryTemperature                = ["37752", 1, "s>", 10, "°C"];

# Unit 1 BatteryPack 1
our $Unit1BatteryPack1SN                    = ["38200", 10, "a*"];
our $Unit1BatteryPack1No                    = ["47750", 1, "S>", 1];
our $Unit1BatteryPack1FirmwareVersion       = ["38210", 15, "a*"];
our $Unit1BatteryPack1WorkingStatus         = ["38228", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit1BatteryPack1Voltage               = ["38235", 1, "S>", 10, "V"];
our $Unit1BatteryPack1Current               = ["38236", 1, "s>", 10, "A"];
our $Unit1BatteryPack1SOC                   = ["38229", 1, "S>", 10, "%"];
our $Unit1BatteryPack1ChargeDischargePower  = ["38233", 2, "l>", 1, "kW"];
our $Unit1BatteryPack1TotalCharge           = ["38238", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack1TotalDischarge        = ["38240", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack1MinimumTemperature    = ["38453", 1, "s>", 10, "°C"];
our $Unit1BatteryPack1MaximumTemperature    = ["38452", 1, "s>", 10, "°C"];

# Unit 1 BatteryPack 2
our $Unit1BatteryPack2SN                    = ["38242", 10, "a*"];
our $Unit1BatteryPack2No                    = ["47751", 1, "S>", 1];
our $Unit1BatteryPack2FirmwareVersion       = ["38252", 15, "a*"];
our $Unit1BatteryPack2WorkingStatus         = ["38270", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit1BatteryPack2Voltage               = ["38277", 1, "S>", 10, "V"];
our $Unit1BatteryPack2Current               = ["38278", 1, "s>", 10, "A"];
our $Unit1BatteryPack2SOC                   = ["38271", 1, "S>", 10, "%"];
our $Unit1BatteryPack2ChargeDischargePower  = ["38275", 2, "l>", 1, "kW"];
our $Unit1BatteryPack2TotalCharge           = ["38280", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack2TotalDischarge        = ["38282", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack2MinimumTemperature    = ["38455", 1, "s>", 10, "°C"];
our $Unit1BatteryPack2MaximumTemperature    = ["38454", 1, "s>", 10, "°C"];

# Unit 1 BatteryPack 3
our $Unit1BatteryPack3SN                    = ["38284", 10, "a*"];
our $Unit1BatteryPack3No                    = ["47752", 1, "S>", 1];
our $Unit1BatteryPack3FirmwareVersion       = ["38294", 15, "a*"];
our $Unit1BatteryPack3WorkingStatus         = ["38312", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit1BatteryPack3Voltage               = ["38319", 1, "S>", 10, "V"];
our $Unit1BatteryPack3Current               = ["38320", 1, "s>", 10, "A"];
our $Unit1BatteryPack3SOC                   = ["38313", 1, "S>", 10, "%"];
our $Unit1BatteryPack3ChargeDischargePower  = ["38317", 2, "l>", 1, "kW"];
our $Unit1BatteryPack3TotalCharge           = ["38322", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack3TotalDischarge        = ["38324", 2, "l>", 100, "kWh"];
our $Unit1BatteryPack3MinimumTemperature    = ["38457", 1, "s>", 10, "°C"];
our $Unit1BatteryPack3MaximumTemperature    = ["38456", 1, "s>", 10, "°C"];

# Unit 2 BatteryPack 1
our $Unit2BatteryPack1SN                    = ["38326", 10, "a*"];
our $Unit2BatteryPack1No                    = ["47753", 1, "S>", 1];
our $Unit2BatteryPack1FirmwareVersion       = ["38336", 15, "a*"];
our $Unit2BatteryPack1WorkingStatus         = ["38354", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit2BatteryPack1Voltage               = ["38361", 1, "S>", 10, "V"];
our $Unit2BatteryPack1Current               = ["38362", 1, "s>", 10, "A"];
our $Unit2BatteryPack1SOC                   = ["38355", 1, "S>", 10, "%"];
our $Unit2BatteryPack1ChargeDischargePower  = ["38359", 2, "l>", 1, "kW"];
our $Unit2BatteryPack1TotalCharge           = ["38364", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack1TotalDischarge        = ["38366", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack1MinimumTemperature    = ["38459", 1, "s>", 10, "°C"];
our $Unit2BatteryPack1MaximumTemperature    = ["38458", 1, "s>", 10, "°C"];

# Unit 2 BatteryPack 2
our $Unit2BatteryPack2SN                    = ["38368", 10, "a*"];
our $Unit2BatteryPack2No                    = ["47754", 1, "S>", 1];
our $Unit2BatteryPack2FirmwareVersion       = ["38378", 15, "a*"];
our $Unit2BatteryPack2WorkingStatus         = ["38396", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit2BatteryPack2Voltage               = ["38403", 1, "S>", 10, "V"];
our $Unit2BatteryPack2Current               = ["38404", 1, "s>", 10, "A"];
our $Unit2BatteryPack2SOC                   = ["38397", 1, "S>", 10, "%"];
our $Unit2BatteryPack2ChargeDischargePower  = ["38401", 2, "l>", 1, "kW"];
our $Unit2BatteryPack2TotalCharge           = ["38406", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack2TotalDischarge        = ["38408", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack2MinimumTemperature    = ["38461", 1, "s>", 10, "°C"];
our $Unit2BatteryPack2MaximumTemperature    = ["38460", 1, "s>", 10, "°C"];

# Unit 2 BatteryPack 3
our $Unit2BatteryPack3SN                    = ["38410", 10, "a*"];
our $Unit2BatteryPack3No                    = ["47755", 1, "S>", 1];
our $Unit2BatteryPack3FirmwareVersion       = ["38420", 15, "a*"];
our $Unit2BatteryPack3WorkingStatus         = ["38438", 1, "S>", 1, undef, undef, \&_running_status];
our $Unit2BatteryPack3Voltage               = ["38445", 1, "S>", 10, "V"];
our $Unit2BatteryPack3Current               = ["38446", 1, "s>", 10, "A"];
our $Unit2BatteryPack3SOC                   = ["38439", 1, "S>", 10, "%"];
our $Unit2BatteryPack3ChargeDischargePower  = ["38443", 2, "l>", 1, "kW"];
our $Unit2BatteryPack3TotalCharge           = ["38448", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack3TotalDischarge        = ["38450", 2, "l>", 100, "kWh"];
our $Unit2BatteryPack3MinimumTemperature    = ["38463", 1, "s>", 10, "°C"];
our $Unit2BatteryPack3MaximumTemperature    = ["38462", 1, "s>", 10, "°C"];

package MODBUS::DATATYPE::Eastron::SDM630;

my @eastron_base = (2, "f>", 1);
my @eastron_fmt  = (undef, sub {$_[0]}, $MODBUS::READ_INPUT_REGISTERS);

our $Phase1LineToNeutralVoltage      = [0x0000, @eastron_base, "V", @eastron_fmt];
our $Phase2LineToNeutralVoltage      = [0x0002, @eastron_base, "V", @eastron_fmt];
our $Phase3LineToNeutralVoltage      = [0x0004, @eastron_base, "V", @eastron_fmt];
our $Phase1Current                   = [0x0006, @eastron_base, "A", @eastron_fmt];
our $Phase2Current                   = [0x0008, @eastron_base, "A", @eastron_fmt];
our $Phase3Current                   = [0x000A, @eastron_base, "A", @eastron_fmt];
our $Phase1Power                     = [0x000C, @eastron_base, "W", @eastron_fmt];
our $Phase2Power                     = [0x000E, @eastron_base, "W", @eastron_fmt];
our $Phase3Power                     = [0x0010, @eastron_base, "W", @eastron_fmt];
our $Phase1VA                        = [0x0012, @eastron_base, "VA", @eastron_fmt];
our $Phase2VA                        = [0x0014, @eastron_base, "VA", @eastron_fmt];
our $Phase3VA                        = [0x0016, @eastron_base, "VA", @eastron_fmt];
our $Phase1VAR                       = [0x0018, @eastron_base, "VAr", @eastron_fmt];
our $Phase2VAR                       = [0x001A, @eastron_base, "VAr", @eastron_fmt];
our $Phase3VAR                       = [0x001C, @eastron_base, "VAr", @eastron_fmt];
our $Phase1PowerFactor               = [0x001E, @eastron_base, "", @eastron_fmt];
our $Phase2PowerFactor               = [0x0020, @eastron_base, "", @eastron_fmt];
our $Phase3PowerFactor               = [0x0022, @eastron_base, "", @eastron_fmt];
our $Phase1Angle                     = [0x0024, @eastron_base, "°", @eastron_fmt];
our $Phase2Angle                     = [0x0026, @eastron_base, "°", @eastron_fmt];
our $Phase3Angle                     = [0x0028, @eastron_base, "°", @eastron_fmt];
our $AverageLineToNeutralVoltage     = [0x002A, @eastron_base, "V", @eastron_fmt];
our $AverageLineCurrent              = [0x002E, @eastron_base, "A", @eastron_fmt];
our $SumOfLineCurrents               = [0x0030, @eastron_base, "A", @eastron_fmt];
our $TotalSystemPower                = [0x0034, @eastron_base, "W", @eastron_fmt];
our $TotalSystemVA                   = [0x0038, @eastron_base, "VA", @eastron_fmt];
our $TotalSystemVAR                  = [0x003C, @eastron_base, "VAr", @eastron_fmt];
our $TotalSystemPowerFactor          = [0x003E, @eastron_base, "", @eastron_fmt];
our $TotalSystemPhaseAngle           = [0x0042, @eastron_base, "°", @eastron_fmt];
our $Frequency                       = [0x0046, @eastron_base, "Hz", @eastron_fmt];
our $TotalImportKWh                  = [0x0048, @eastron_base, "kWh", @eastron_fmt];
our $TotalExportKWh                  = [0x004A, @eastron_base, "kWh", @eastron_fmt];
our $TotalImportKVarh                = [0x004C, @eastron_base, "kVArh", @eastron_fmt];
our $TotalExportKVarh                = [0x004E, @eastron_base, "kVArh", @eastron_fmt];
our $TotalVAh                        = [0x0050, @eastron_base, "VAh", @eastron_fmt];
our $Ah                              = [0x0052, @eastron_base, "Ah", @eastron_fmt];
our $TotalSystemPowerDemand          = [0x0054, @eastron_base, "W", @eastron_fmt];
our $MaximumTotalSystemPowerDemand   = [0x0056, @eastron_base, "W", @eastron_fmt];
our $TotalSystemVADemand             = [0x0064, @eastron_base, "VA", @eastron_fmt];
our $MaximumTotalSystemVADemand      = [0x0066, @eastron_base, "VA", @eastron_fmt];
our $NeutralCurrentDemand            = [0x0068, @eastron_base, "A", @eastron_fmt];
our $MaximumNeutralCurrentDemand     = [0x006A, @eastron_base, "A", @eastron_fmt];
our $Line1ToLine2Voltage             = [0x00C8, @eastron_base, "V", @eastron_fmt];
our $Line2ToLine3Voltage             = [0x00CA, @eastron_base, "V", @eastron_fmt];
our $Line3ToLine1Voltage             = [0x00CC, @eastron_base, "V", @eastron_fmt];
our $AverageLineToLineVoltage        = [0x00CE, @eastron_base, "V", @eastron_fmt];
our $NeutralCurrent                  = [0x00E0, @eastron_base, "A", @eastron_fmt];
our $Phase1LNVoltageTHD              = [0x00EA, @eastron_base, "V", @eastron_fmt];
our $Phase2LNVoltageTHD              = [0x00EC, @eastron_base, "V", @eastron_fmt];
our $Phase3LNVoltageTHD              = [0x00EE, @eastron_base, "V", @eastron_fmt];
our $Phase1CurrentTHD                = [0x00F0, @eastron_base, "A", @eastron_fmt];
our $Phase2CurrentTHD                = [0x00F2, @eastron_base, "A", @eastron_fmt];
our $Phase3CurrentTHD                = [0x00F4, @eastron_base, "A", @eastron_fmt];
our $AverageLineToNeutralVoltageTHD  = [0x00F8, @eastron_base, "V", @eastron_fmt];
our $AverageLineCurrentTHD           = [0x00FA, @eastron_base, "A", @eastron_fmt];
our $Phase1CurrentDemand             = [0x0102, @eastron_base, "A", @eastron_fmt];
our $Phase2CurrentDemand             = [0x0104, @eastron_base, "A", @eastron_fmt];
our $Phase3CurrentDemand             = [0x0106, @eastron_base, "A", @eastron_fmt];
our $MaximumPhase1CurrentDemand      = [0x0108, @eastron_base, "A", @eastron_fmt];
our $MaximumPhase2CurrentDemand      = [0x010A, @eastron_base, "A", @eastron_fmt];
our $MaximumPhase3CurrentDemand      = [0x010C, @eastron_base, "A", @eastron_fmt];
our $Line1ToLine2VoltageTHD          = [0x014E, @eastron_base, "V", @eastron_fmt];
our $Line2ToLine3VoltageTHD          = [0x0150, @eastron_base, "V", @eastron_fmt];
our $Line3ToLine1VoltageTHD          = [0x0152, @eastron_base, "V", @eastron_fmt];
our $AverageLineToLineVoltageTHD     = [0x0154, @eastron_base, "V", @eastron_fmt];
our $TotalKWh                        = [0x0156, @eastron_base, "kWh", @eastron_fmt];
our $TotalKVarh                      = [0x0158, @eastron_base, "kVArh", @eastron_fmt];
our $L1ImportKWh                     = [0x015A, @eastron_base, "kWh", @eastron_fmt];
our $L2ImportKWh                     = [0x015C, @eastron_base, "kWh", @eastron_fmt];
our $L3ImportKWh                     = [0x015E, @eastron_base, "kWh", @eastron_fmt];
our $L1ExportKWh                     = [0x0160, @eastron_base, "kWh", @eastron_fmt];
our $L2ExportKWh                     = [0x0162, @eastron_base, "kWh", @eastron_fmt];
our $L3ExportKWh                     = [0x0164, @eastron_base, "kWh", @eastron_fmt];
our $L1TotalKWh                      = [0x0166, @eastron_base, "kWh", @eastron_fmt];
our $L2TotalKWh                      = [0x0168, @eastron_base, "kWh", @eastron_fmt];
our $L3TotalKWh                      = [0x016A, @eastron_base, "kWh", @eastron_fmt];
our $L1ImportKVarh                   = [0x016C, @eastron_base, "kVArh", @eastron_fmt];
our $L2ImportKVarh                   = [0x016E, @eastron_base, "kVArh", @eastron_fmt];
our $L3ImportKVarh                   = [0x0170, @eastron_base, "kVArh", @eastron_fmt];
our $L1ExportKVarh                   = [0x0172, @eastron_base, "kVArh", @eastron_fmt];
our $L2ExportKVarh                   = [0x0174, @eastron_base, "kVArh", @eastron_fmt];
our $L3ExportKVarh                   = [0x0176, @eastron_base, "kVArh", @eastron_fmt];
our $L1TotalKVarh                    = [0x0178, @eastron_base, "kVArh", @eastron_fmt];
our $L2TotalKVarh                    = [0x017A, @eastron_base, "kVArh", @eastron_fmt];
our $L3TotalKVarh                    = [0x017C, @eastron_base, "kVArh", @eastron_fmt];

our $SystemType                      = [0x000A, 2, "f>", 1, undef, undef, sub {$_[0] == 1 ? "1p2w" : $_[0] == 2 ? "3p3w" : $_[0] == 3 ? "3p4w" : ""}, $MODBUS::READ_HOLDING_REGISTERS];
our $BaudRate                        = [0x001C, 2, "f>", 1, undef, undef, sub {$_[0]}, $MODBUS::READ_HOLDING_REGISTERS];

package MODBUS::DATATYPE::Eastron::SDM230;

our $LineToNeutralVoltage                  = [0x0000, @eastron_base, "V", @eastron_fmt];
our $LineCurrent                           = [0x0006, @eastron_base, "A", @eastron_fmt];
our $Power                                 = [0x000C, @eastron_base, "W", @eastron_fmt];
our $VA                                    = [0x0012, @eastron_base, "VA", @eastron_fmt];
our $VAR                                   = [0x0018, @eastron_base, "VAr", @eastron_fmt];
our $PowerFactor                           = [0x001E, @eastron_base, "", @eastron_fmt];
our $Frequency                             = [0x0046, @eastron_base, "Hz", @eastron_fmt];
our $ImportKWh                             = [0x0048, @eastron_base, "kWh", @eastron_fmt];
our $ExportKWh                             = [0x004A, @eastron_base, "kWh", @eastron_fmt];
our $ImportKVarh                           = [0x004C, @eastron_base, "kVArh", @eastron_fmt];
our $ExportKVarh                           = [0x004E, @eastron_base, "kVArh", @eastron_fmt];
our $TotalSystemPower                      = [0x0054, @eastron_base, "W", @eastron_fmt];
our $MaximumTotalSystemPower               = [0x0056, @eastron_base, "W", @eastron_fmt];
our $CurrentSystemPositivePower            = [0x0058, @eastron_base, "W", @eastron_fmt];
our $MaximumSytemPositivePower             = [0x005A, @eastron_base, "W", @eastron_fmt];
our $CurrentSystemReversePower             = [0x005C, @eastron_base, "W", @eastron_fmt];
our $MaximumSystemReversePower             = [0x005E, @eastron_base, "W", @eastron_fmt];
our $Current                               = [0x0102, @eastron_base, "A", @eastron_fmt];
our $MaximumCurrent                        = [0x0108, @eastron_base, "A", @eastron_fmt];
our $TotalKWh                              = [0x0156, @eastron_base, "kWh", @eastron_fmt];
our $TotalKVarh                            = [0x0158, @eastron_base, "kVArh", @eastron_fmt];
our $CurrentResettableTotalActiveEnergy    = [0x0180, @eastron_base, "kWh", @eastron_fmt];
our $CurrentResettableTotalReactiveEnergy  = [0x0182, @eastron_base, "kVArh", @eastron_fmt];


package main;
}
