#!/usr/bin/perl

# ip6tables -I OUTPUT 1 -d ::0/0 \
#   -p tcp -m multiport --ports 57875 \
#   -j NFQUEUE --queue-num 121 --queue-bypass
use strict; use warnings;

use Socket qw(AF_UNSPEC AF_INET AF_INET6 SOCK_RAW SOCK_DGRAM inet_ntoa);
use Socket6 qw(inet_ntop inet_pton);
use POSIX ();
use Fcntl qw(F_GETFL F_SETFL O_NONBLOCK);

our $AF_NETLINK = 16;

our $NETLINK_NETFILTER        = 12;
our $NETLINK_ADD_MEMBERSHIP   = 1;
our $NETLINK_DROP_MEMBERSHIP  = 2;
our $NETLINK_PKTINFO          = 3;
our $NETLINK_BROADCAST_ERROR  = 4;
our $NETLINK_NO_ENOBUFS       = 5;
our $NETLINK_LISTEN_ALL_NSID  = 8;
our $NETLINK_LIST_MEMBERSHIPS = 9;
our $NETLINK_CAP_ACK          = 10;
our $NETLINK_EXT_ACK          = 11;
our $NETLINK_GET_STRICT_CHK   = 12;

our $NETLINK_TAX_NONE   = 0x00000000;  # NETLINK_TAX_NONE
our $NETLINK_TAX_NET    = 0x00000001;  # NETLINK_TAX_NET
our $NETLINK_TAX_INET   = 0x00000002;  # NETLINK_TAX_INET
our $NFNL_SUBSYS_QUEUE  = 3;

our $NFNETLINK_V0 = 0;

our $NFQNL_MSG_PACKET        = 0;
our $NFQNL_MSG_VERDICT       = 1;
our $NFQNL_MSG_CONFIG        = 2;
our $NFQNL_MSG_VERDICT_BATCH = 3;

our $NFQNL_CFG_CMD_NONE      = 0;
our $NFQNL_CFG_CMD_BIND      = 1;
our $NFQNL_CFG_CMD_UNBIND    = 2;
our $NFQNL_CFG_CMD_PF_BIND   = 3;
our $NFQNL_CFG_CMD_PF_UNBIND = 4;

our $NFQNL_COPY_NONE    = 0;
our $NFQNL_COPY_META    = 1;
our $NFQNL_COPY_PACKET  = 2;

our $NFQA_CFG_UNSPEC       = 0;
our $NFQA_CFG_CMD          = 1;
our $NFQA_CFG_PARAMS       = 2;
our $NFQA_CFG_QUEUE_MAXLEN = 3;
our $NFQA_CFG_MASK         = 4;
our $NFQA_CFG_FLAGS        = 5;

our $NFQA_CFG_F_FAIL_OPEN = 0x01;
our $NFQA_CFG_F_CONNTRACK = 0x02;
our $NFQA_CFG_F_GSO       = 0x04;
our $NFQA_CFG_F_UID_GID   = 0x08;
our $NFQA_CFG_F_SECCTX    = 0x16;
our $NFQA_CFG_F_MAX       = 0x32;

# GENERIC Flags
our $NLM_F_REQUEST       = 0x01;
our $NLM_F_MULTI         = 0x02;
our $NLM_F_ACK           = 0x04;
our $NLM_F_ECHO          = 0x08;
our $NLM_F_DUMP_INTR     = 0x10;
our $NLM_F_DUMP_FILTERED = 0x20;

# MODIFIERS for GET request
our $NLM_F_ROOT          = 0x100;
our $NLM_F_MATCH         = 0x200;
our $NLM_F_ATOMIC        = 0x400;
our $NLM_F_DUMP          = $NLM_F_ROOT | $NLM_F_MATCH;

# MODIFIERS for NEW request
our $NLM_F_REPLACE       = 0x100;
our $NLM_F_EXCL          = 0x200;
our $NLM_F_CREATE        = 0x400;
our $NLM_F_APPEND        = 0x800;

# MODIFIERS for DELETE request
our $NLM_F_NONREC        = 0x100;

# Flags for ACK message
our $NLM_F_CAPPED        = 0x40;
our $NLM_F_ACK_TLVS      = 0x80;

our $NFQA_PACKET_HDR         = 1;
our $NFQA_VERDICT_HDR        = 2;
our $NFQA_MARK               = 3;
our $NFQA_TIMESTAMP          = 4;
our $NFQA_IFINDEX_INDEV      = 5;
our $NFQA_IFINDEX_OUTDEV     = 6;
our $NFQA_IFINDEX_PHYSINDEV  = 7;
our $NFQA_IFINDEX_PHYSOUTDEV = 8;
our $NFQA_HWADDR             = 9;
our $NFQA_PAYLOAD            = 10;
our $NFQA_CT                 = 11;
our $NFQA_CT_INFO            = 12;
our $NFQA_CAP_LEN            = 13;
our $NFQA_SKB_INFO           = 14;
our $NFQA_EXP                = 15;
our $NFQA_UID                = 16;
our $NFQA_GID                = 17;
our $NFQA_SECCTX             = 18;
our $NFQA_VLAN               = 19;
our $NFQA_L2HDR              = 20;


our $NLMSG_NOOP    = 1;
our $NLMSG_ERROR   = 2;
our $NLMSG_DONE    = 3;
our $NLMSG_OVERRUN = 4;

our $NF_DROP   = 0;
our $NF_ACCEPT = 1;
our $NF_STOLEN = 2;
our $NF_QUEUE  = 3;
our $NF_REPEAT = 4;


our $SOL_NETLINK = 270;

if(-f $ARGV[0]){
    open(my $fh, "<", $ARGV[0])
        or die "Can't open $ARGV[0]: $!";
    my $data = do {local $/; <$fh>};
    close($fh);
    # PCAP global header
    my $pcap_hdr = substr($data, 0, 24, '');
    while(length($data) > 0){
        my $pkt_hdr = substr($data, 0, 16, '');
        last unless length($pkt_hdr) == 16;
        # PCAP record header
        my ($ts_sec, $ts_usec, $incl_len, $orig_len) = unpack("LLLL", $pkt_hdr);
        log_debug("ts_sec: $ts_sec");
        log_debug("ts_usec: $ts_usec");
        log_debug("incl_len: $incl_len");
        log_debug("orig_len: $orig_len");

        # Ethernet header
        my $ethhdr = substr($data, 0, 14, '');
        my ($eth_dst, $eth_src, $eth_type) = unpack("a6a6S>", $ethhdr);
        log_debug("eth_dst: ".to_hex($eth_dst));
        log_debug("eth_src: ".to_hex($eth_src));
        log_debug("eth_type: ".to_hex(pack("S>", $eth_type)));
        my $pkt = substr($data, 0, $incl_len-14, '');
        # Ethernet type?
        if($eth_type == 0x0800){
            log_debug("IP");
            my $data_payload = parse_ip_packet(\$pkt);
            if(length($pkt) > 0){
                log_debug("Remaining: ".length($pkt));
                log_debug("Remaining[hex]: ".to_hex($pkt));
            }
            print $data_payload if length($data_payload//"");
        } elsif($eth_type == 0x0806){
            log_debug("ARP");
        } else {
            log_debug("Unknown eth_type: ".to_hex(pack("S>",$eth_type)));
        }
        #my $eth_check = unpack("L>", substr($data, 0, 2, ''));
        #log_debug("eth_check: $eth_check") if $eth_check;
    }
    exit;
}

my $queue_num = shift @ARGV // die "Usage: $0 QUEUE_NUM";
my $my_id = 0;#$$;

socket(my $nf_fh, $AF_NETLINK, SOCK_RAW, $NETLINK_NETFILTER)
    // die "socket: $!";
my $bind_addr = pack("SSLL", $AF_NETLINK, 0, $my_id, 0);
bind($nf_fh, $bind_addr)
    // die "bind: $!";
binmode($nf_fh);
my $s_flags = fcntl($nf_fh, F_GETFL, 0)
    or die "Can't get flags for the socket: $!";
fcntl($nf_fh, F_SETFL, $s_flags|O_NONBLOCK)
    or die "Can't set flags for the socket: $!";
setsockopt($nf_fh, $SOL_NETLINK, $NETLINK_EXT_ACK, 0)
    or die "setsockopt: $!";
setsockopt($nf_fh, $SOL_NETLINK, $NETLINK_CAP_ACK, 0)
    or die "setsockopt: $!";
setsockopt($nf_fh, $SOL_NETLINK, $NETLINK_NO_ENOBUFS, 1)
    or die "setsockopt: $!";

# register for packets from the queue, $queue_num
nfqnl_send($nf_fh, $bind_addr,
    nfqnl_bind(AF_INET, $queue_num),
    nfqnl_copy_packet($queue_num, 0xffff) # full size copy
) or die "nfqnl_send: $!";

log_info("Listening for packets on queue $queue_num");
M_LOOP:
while(1){
    local $!;
    # is there new data?
    my $rin = '';
    vec($rin, fileno($nf_fh), 1) = 1;
    select(my $rout = $rin, undef, undef, undef)
        or !($!{EINTR} or $!{EAGAIN}) and next M_LOOP or die "select: $!";
    next unless vec($rout, fileno($nf_fh), 1);
    # read data, process the netlink/netfilter/queue message
    while(1){
        local $!;
        my $r = recv($nf_fh, my $pkt_msg, 1_000_000, 0);
        if(!defined $r){
            next M_LOOP if $!{EAGAIN};
            die "recv problem for: $!";
        }
        my ($m_seq, $m_pid, $pktids) = handle_nlmsg($pkt_msg);
        # send positive verdict
        foreach my $pkt_id (@$pktids){
            log_info("Sending verdict for packet $pkt_id");
            nfqnl_send($nf_fh, $bind_addr, nfqnl_msg_verdict($m_seq, $m_pid, $queue_num, $pkt_id, $NF_ACCEPT))
                or die "nfqnl_send: $!";
        }
    }
}

close($nf_fh);
exit 0;

sub nfqnl_send {
    my ($nf_fh, $tgt_addr, @cmds) = @_;
    foreach my $msg (@cmds){
        send($nf_fh, $msg, 0, $tgt_addr)
            or die "send: $!";
    }
    return !$!;
}

sub nfqnl_bind {
    my ($pf_family, $res_id) = @_;
    my $m_hdr = nlmsghdr($NFNL_SUBSYS_QUEUE<<8|$NFQNL_MSG_CONFIG, $NLM_F_REQUEST);
    my $p_hdr = genlmsghdr(AF_UNSPEC, $NFNETLINK_V0, $res_id);
    return nlmsg($m_hdr, $p_hdr,
        nlattr($NFQA_CFG_CMD, nfqnl_msg_config_cmd($NFQNL_CFG_CMD_BIND, $pf_family))
    );
}

sub nfqnl_copy_packet {
    my ($res_id, $pkt_size) = @_;
    $pkt_size //= 0xffff;
    my $m_hdr = nlmsghdr($NFNL_SUBSYS_QUEUE<<8|$NFQNL_MSG_CONFIG, $NLM_F_REQUEST);
    my $p_hdr = genlmsghdr(AF_UNSPEC, $NFNETLINK_V0, $res_id);
    return nlmsg($m_hdr, $p_hdr,
        nlattr($NFQA_CFG_PARAMS, pack("L>C", $pkt_size, $NFQNL_COPY_PACKET)),
        nlattr($NFQA_CFG_FLAGS, pack("L>", $NFQA_CFG_F_GSO)),
        nlattr($NFQA_CFG_MASK, pack("L>", $NFQA_CFG_F_GSO)),
    );
}

sub nfqnl_msg_verdict {
    my ($nl_seq, $nl_pid, $res_id, $pkt_id, $verdict) = @_;
    my $m_hdr = nlmsghdr($NFNL_SUBSYS_QUEUE<<8|$NFQNL_MSG_VERDICT, $NLM_F_REQUEST, $nl_seq, $nl_pid);
    my $p_hdr = genlmsghdr(AF_UNSPEC, $NFNETLINK_V0, $res_id);
    return nlmsg($m_hdr, $p_hdr,
        nlattr($NFQA_VERDICT_HDR, pack("L>L>", $verdict, $pkt_id)),
    );
}

sub nlmsg {
    my ($m_hdr, $p_hdr, @nlattr) = @_;
    my $t_msg .= $m_hdr.$p_hdr;
    $t_msg .= "\0" x (length($t_msg) % 4 ?4 - length($t_msg) % 4: 0);
    foreach my $nlattr (@nlattr){
        $t_msg .= $nlattr;
        $t_msg .= "\0" x (length($t_msg) % 4 ?4 - length($t_msg) % 4: 0);
    }
    return pack("L<a*", length($t_msg)+4, $t_msg);
}

sub nfqnl_msg_config_cmd {
    my ($cmd, $pf) = @_;
    return pack("CCS>", $cmd, 0, $pf);
}

sub genlmsghdr {
    my ($nf_family, $version, $res_id) = @_;
    return pack("CCS>", $nf_family, $version, $res_id);
}

sub nlattr {
    my ($nla_type, $data) = @_;
    my $tlv = pack("SSa*", length($data)+4, $nla_type, $data);
    $tlv .= "\0" x (length($tlv) % 4 ?4 - length($tlv) % 4: 0);
    return $tlv;
}

sub nlmsghdr {
    my ($nl_type, $nl_flags, $nl_seq, $nl_pid) = @_;
    $nl_flags //= 0;
    $nl_seq   //= 0;
    $nl_pid   //= $my_id;
    return pack("SSLL", $nl_type, $nl_flags, $nl_seq, $nl_pid);
}

sub handle_nlmsg {
    my ($msg) = @_;
    my ($len, $type, $flags, $seq, $pid, $data) = unpack("LSSLLa*", $msg);
    log_debug("MSG[".length($msg)."]: len: $len, type: $type, flags: $flags, seq: $seq, pid: $pid");
    if($type == $NLMSG_ERROR){
        my $err_num = unpack("La*", $data);
        die "ERROR[$err_num]";
    }
    if($flags & $NLM_F_ACK){
        log_debug("ACK");
        return;
    }
    my $ret;
    if($type == ($NFNL_SUBSYS_QUEUE<<8|$NFQNL_MSG_PACKET)){
        $ret = handle_nfqnl_msg_packet($data);
    } else {
        log_debug("ERROR: Unknown message type: $type");
    }
    return $seq, $pid, $ret;
}

sub handle_nfqnl_msg_packet {
    my ($data) = @_;
    my ($gen_hdr_nf_family, $gen_hdr_nf_version, $gen_hdr_nf_res_id, $nl_attrs) = unpack("CCS>a*", $data);
    log_debug("gen_hdr_nf_family: $gen_hdr_nf_family");
    log_debug("gen_hdr_nf_version: $gen_hdr_nf_version");
    log_debug("queue_number $gen_hdr_nf_res_id");
    my @pkt_ids;
    while(length($nl_attrs) > 0){
        log_debug(" hex: ".to_hex($nl_attrs)."");
        my ($nla_len, $nla_type) = unpack("SS", $nl_attrs);
        my $nr_pad = $nla_len % 4 ?4 - $nla_len % 4: 0;
        my $nla_data = substr($nl_attrs, 0, $nla_len, '');
        substr($nla_data, 0, 4, '');
        log_debug(" nr_pad: $nr_pad");
        log_debug(" nla_type: $nla_type");
        log_debug(" nla_len: $nla_len");
        log_debug(" nla_data: ".to_hex($nla_data)."");
        log_debug(" remaining: ".length($nl_attrs)."");
        if($nla_type == $NFQA_PACKET_HDR){
            my ($packet_id, $hw_protocol, $hook) = unpack("L>S>C", $nla_data);
            log_debug(" - packet_id: $packet_id");
            log_debug(" - hw_protocol: $hw_protocol");
            log_debug(" - hook: $hook");
            push @pkt_ids, $packet_id;
        } elsif($nla_type == $NFQA_HWADDR){
            my ($hw_addrlen, $pad) = unpack("S>S", $nla_data);
            my $hw_addr = substr($nla_data, 0, $hw_addrlen, '');
            log_debug(" - hw_addrlen: $hw_addrlen");
            log_debug(" - hw_addr: ".to_hex($hw_addr)."");
        } elsif($nla_type == $NFQA_TIMESTAMP){
            my ($sec, $usec) = unpack("Q>Q>", $nla_data);
            log_debug(" - sec: $sec");
            log_debug(" - usec: $usec");
        } elsif($nla_type == $NFQA_IFINDEX_INDEV){
            my $ifindex = unpack("L>", $nla_data);
            log_debug(" - ifindex: $ifindex");
        } elsif($nla_type == $NFQA_IFINDEX_OUTDEV){
            my $ifindex = unpack("L>", $nla_data);
            log_debug(" - ifindex: $ifindex");
        } elsif($nla_type == $NFQA_IFINDEX_PHYSINDEV){
            my $ifindex = unpack("L>", $nla_data);
            log_debug(" - ifindex: $ifindex");
        } elsif($nla_type == $NFQA_IFINDEX_PHYSOUTDEV){
            my $ifindex = unpack("L>", $nla_data);
            log_debug(" - ifindex: $ifindex");
        } elsif($nla_type == $NFQA_PAYLOAD){
            log_debug(" - payload[hex]: ".to_hex($nla_data)."");
            my $p_data = parse_ip_packet(\$nla_data);
            if(defined $p_data){
                print $p_data;
            }
        } elsif($nla_type == $NFQA_CT){
            log_debug(" - conntrack[hex]: ".to_hex($nla_data)."");
        } elsif($nla_type == $NFQA_CT_INFO){
            log_debug(" - conntrack_info[hex]: ".to_hex($nla_data)."");
        } elsif($nla_type == $NFQA_MARK){
            my $mark = unpack("L>", $nla_data);
            log_debug(" - mark: $mark");
        } elsif($nla_type == $NFQA_VLAN){
            my ($vlan_tci, $vlan_proto) = unpack("S>S>", $nla_data);
            log_debug(" - vlan_tci: $vlan_tci");
            log_debug(" - vlan_proto: $vlan_proto");
        } elsif($nla_type == $NFQA_L2HDR){
            log_debug(" - l2hdr[hex]: ".to_hex($nla_data)."");
        } elsif($nla_type == $NFQA_EXP){
            log_debug(" - exp[hex]: ".to_hex($nla_data)."");
        } elsif($nla_type == $NFQA_UID){
            my $uid = unpack("L>", $nla_data);
            log_debug(" - uid: $uid");
        } elsif($nla_type == $NFQA_GID){
            my $gid = unpack("L>", $nla_data);
            log_debug(" - gid: $gid");
        } elsif($nla_type == $NFQA_SECCTX){
            log_debug(" - secctx[hex]: ".to_hex($nla_data)."");
        } elsif($nla_type == $NFQA_SKB_INFO){
            log_debug(" - skb_info[hex]: ".to_hex($nla_data)."");
        } else {
            log_debug(" - unknown $nla_type");
        }
        substr($nl_attrs, 0, $nr_pad, '');
    }
    return \@pkt_ids;
}

sub to_hex {
    my $b = \$_[0];
    return join("",map {sprintf("%02x", ord $_)} split '', $$b//"");
}

sub log_info {
    my ($msg) = @_;
    print STDERR "INFO: $msg\n";
    return;
}

sub log_debug {
    my ($msg) = @_;
    print STDERR "DEBUG: $msg\n" if $ENV{DEBUG};
    return;
}

sub parse_ip_packet {
    my ($nla_data) = @_;
    my $ip_version = unpack("C", $$nla_data);
    my ($iphdr, $iplen, $ipproto);
    if(($ip_version & 0xf0) == 0x40){
        my $ip_ihl = $ip_version & 0x0f;
        my $ip_hdr_size = $ip_ihl * 4;
        $iphdr = unpack("a".($ip_ihl*4), substr($$nla_data, 0, $ip_hdr_size, ''));
        log_debug(" - iphdr[hex]: ".to_hex($iphdr)."");
        # let's parse the ipv4 header
        my ($ip_version_ihl, $ip_dscp_ecn, $ip_tot_len, $ip_id, $ip_flags_fragment_offset, $ip_ttl, $ip_protocol, $ip_check, $ip_saddr, $ip_daddr) = unpack("CCS>S>S>CCa2a4a4", $iphdr);
        log_debug(" - ip_version: $ip_version");
        log_debug(" - ip_ihl: $ip_ihl");
        log_debug(" - ip_dscp_ecn: $ip_dscp_ecn");
        log_debug(" - ip_tot_len: $ip_tot_len");
        log_debug(" - ip_id: $ip_id");
        log_debug(" - ip_flags_fragment_offset: $ip_flags_fragment_offset");
        log_debug(" - ip_ttl: $ip_ttl");
        log_debug(" - ip_protocol: $ip_protocol");
        log_debug(" - ip_check: ".to_hex($ip_check));
        log_debug(" - ip_saddr: ".inet_ntoa($ip_saddr)."");
        log_debug(" - ip_daddr: ".inet_ntoa($ip_daddr)."");
        my $ip_options = substr($iphdr, 20, $ip_ihl*4, '')
            if $ip_ihl > 5;
        $ipproto = $ip_protocol;
        $iplen   = $ip_tot_len;
    } elsif(($ip_version & 0xf0) == 0x60){
        $iphdr = unpack("a40", substr($$nla_data, 0, 40, ''));
        log_debug(" - ipv6hdr[hex]: ".to_hex($iphdr)."");
        # let's parse the ipv6 header
        my ($ip6_version, $ip6_traffic_class, $ip6_flow_label, $ip6_payload_len, $ip6_next_header, $ip6_hop_limit, $ip6_src, $ip6_dst) = unpack("CCS>S>CCa16a16", $iphdr);
        $ip6_version &= 0xff;
        log_debug(" - ip6_version: $ip6_version");
        log_debug(" - ip6_traffic_class: $ip6_traffic_class");
        log_debug(" - ip6_flow_label: $ip6_flow_label");
        log_debug(" - ip6_payload_len: $ip6_payload_len");
        log_debug(" - ip6_next_header: $ip6_next_header");
        log_debug(" - ip6_hop_limit: $ip6_hop_limit");
        log_debug(" - ip6_src: ".inet_ntop(AF_INET6, $ip6_src)."");
        log_debug(" - ip6_dst: ".inet_ntop(AF_INET6, $ip6_dst)."");
        $ipproto = $ip6_next_header;
        $iplen   = $ip6_payload_len;
    } else {
        log_debug(" - unknown ip_version: $ip_version:".($ip_version & 0xff)."");
        return;
    }
    if($ipproto == 6){ # TCP
        my $tcphdr = substr($$nla_data, 0, 20, '');
        # now we parse the tcp header
        log_debug(" - tcphdr[hex]: ".to_hex($tcphdr)."");
        my ($tcp_sport, $tcp_dport, $tcp_seq, $tcp_ack_seq, $tcp_data_off_res, $tcp_flags, $tcp_window, $tcp_check, $tcp_urg_ptr) = unpack("S>S>L>L>CCS>a2S>", $tcphdr);
        my $tcp_data_off = $tcp_data_off_res >> 4;
        my $tcp_res = $tcp_data_off_res & 0x0f;
        log_debug(" - tcp_sport: $tcp_sport");
        log_debug(" - tcp_dport: $tcp_dport");
        log_debug(" - tcp_seq: $tcp_seq");
        log_debug(" - tcp_ack_seq: $tcp_ack_seq");
        log_debug(" - tcp_data_off: $tcp_data_off");
        log_debug(" - tcp_res: $tcp_res");
        log_debug(" - tcp_flags: $tcp_flags");
        # and parse the flags
        my $tcp_fin = $tcp_flags & 0x01;
        my $tcp_syn = ($tcp_flags & 0x02) >> 1;
        my $tcp_rst = ($tcp_flags & 0x04) >> 2;
        my $tcp_psh = ($tcp_flags & 0x08) >> 3;
        my $tcp_ack = ($tcp_flags & 0x10) >> 4;
        my $tcp_urg = ($tcp_flags & 0x20) >> 5;
        my $tcp_ece = ($tcp_flags & 0x40) >> 6;
        my $tcp_cwr = ($tcp_flags & 0x80) >> 7;
        log_debug(" - tcp_fin: $tcp_fin");
        log_debug(" - tcp_syn: $tcp_syn");
        log_debug(" - tcp_rst: $tcp_rst");
        log_debug(" - tcp_psh: $tcp_psh");
        log_debug(" - tcp_ack: $tcp_ack");
        log_debug(" - tcp_urg: $tcp_urg");
        log_debug(" - tcp_ece: $tcp_ece");
        log_debug(" - tcp_cwr: $tcp_cwr");
        log_debug(" - tcp_window: $tcp_window");
        log_debug(" - tcp_check: ".to_hex($tcp_check));
        log_debug(" - tcp_urg_ptr: $tcp_urg_ptr");
        my $tcp_options = substr($$nla_data, 0, $tcp_data_off*4-20, '')
            if $tcp_data_off > 5;
        my $p_size = $iplen - $tcp_data_off*4 - length($iphdr);
        if($p_size > 0){
            my $payload = substr($$nla_data, 0, $p_size, '');
            log_debug(" - payload_size: $p_size");
            log_debug(" - payload[hex]: ".to_hex($payload));
            log_debug(" - payload[raw]: ".$payload);
            if($tcp_psh){
                return $payload;
            }
        }
    } elsif($ipproto == 1) { # ICMP
        my $icmphdr = substr($$nla_data, 0, 8, '');
        # now we parse the icmp header
        log_debug(" - icmphdr[hex]: ".to_hex($icmphdr)."");
        my ($icmp_type, $icmp_code, $icmp_check, $icmp_payload) = unpack("CCS>a4", $icmphdr);
        log_debug(" - icmp_type: $icmp_type");
        log_debug(" - icmp_code: $icmp_code");
        log_debug(" - icmp_check: ".to_hex($icmp_check));
        log_debug(" - icmp_payload[hex]: ".to_hex($icmp_payload));
    } elsif($ipproto == 0){ # IPv6 HOPOPT
    } elsif($ipproto == 2){ # IGMP
    } elsif($ipproto == 17){ # UDP
    } elsif($ipproto == 41){ # IPv6 ENCAP
    } elsif($ipproto == 43){ # IPv6 Route
    } elsif($ipproto == 44){ # IPv6 Frag
    } elsif($ipproto == 50){ # ESP
    } elsif($ipproto == 51){ # AH
    } elsif($ipproto == 58){ # ICMPv6
    } elsif($ipproto == 59){ # No Next Header
    } elsif($ipproto == 60){ # Destination Options
    } elsif($ipproto == 103){ # PIM
    } elsif($ipproto == 132){ # SCTP
    } elsif($ipproto == 133){ # FC
    } elsif($ipproto == 135){ # Mobility Header
    } elsif($ipproto == 139){ # HIP
    } elsif($ipproto == 140){ # Shim6
    } elsif($ipproto == 141){ # WESP
    } elsif($ipproto == 142){ # ROHC
    } else {
        log_debug(" - unknown ipproto: $ipproto");
    }
    return;
}
