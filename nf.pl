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

my $queue_num = shift @ARGV // die "Usage: $0 QUEUE_NUM\n";
my $my_id = 0;#$$;

socket(my $nf_fh, $AF_NETLINK, SOCK_RAW, $NETLINK_NETFILTER)
    // die "socket: $!";
my $bind_addr = pack("SSLL", $AF_NETLINK, 0, $my_id, 0);
bind($nf_fh, $bind_addr)
    // die "bind: $!";
binmode($nf_fh);
my $s_flags = fcntl($nf_fh, F_GETFL, 0)
    or die "Can't get flags for the socket: $!\n";
fcntl($nf_fh, F_SETFL, $s_flags|O_NONBLOCK)
    or die "Can't set flags for the socket: $!\n";
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

print "Listening for packets on queue $queue_num\n";
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
            die "recv problem for: $!\n";
        }
        my ($m_seq, $m_pid, $pktids) = handle_nlmsg($pkt_msg);
        # send positive verdict
        foreach my $pkt_id (@$pktids){
            print "Sending verdict for packet $pkt_id\n";
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
    print "MSG[".length($msg)."]: len: $len, type: $type, flags: $flags, seq: $seq, pid: $pid\n";
    if($type == $NLMSG_ERROR){
        my $err_num = unpack("La*", $data);
        die "ERROR[$err_num]\n";
    }
    if($flags & $NLM_F_ACK){
        print "ACK\n";
        return;
    }
    my $ret;
    if($type == ($NFNL_SUBSYS_QUEUE<<8|$NFQNL_MSG_PACKET)){
        $ret = handle_nfqnl_msg_packet($data);
    } else {
        print "ERROR: Unknown message type: $type\n";
    }
    return $seq, $pid, $ret;
}

sub handle_nfqnl_msg_packet {
    my ($data) = @_;
    my ($gen_hdr_nf_family, $gen_hdr_nf_version, $gen_hdr_nf_res_id, $nl_attrs) = unpack("CCS>a*", $data);
    print "gen_hdr_nf_family: $gen_hdr_nf_family\n";
    print "gen_hdr_nf_version: $gen_hdr_nf_version\n";
    print "queue_number $gen_hdr_nf_res_id\n";
    my @pkt_ids;
    while(length($nl_attrs) > 0){
        print " hex: ".to_hex($nl_attrs)."\n";
        my ($nla_len, $nla_type) = unpack("SS", $nl_attrs);
        my $nr_pad = $nla_len % 4 ?4 - $nla_len % 4: 0;
        my $nla_data = substr($nl_attrs, 0, $nla_len, '');
        substr($nla_data, 0, 4, '');
        print " nr_pad: $nr_pad\n";
        print " nla_type: $nla_type\n";
        print " nla_len: $nla_len\n";
        print " nla_data: ".to_hex($nla_data)."\n";
        print " remaining: ".length($nl_attrs)."\n";
        if($nla_type == $NFQA_PACKET_HDR){
            my ($packet_id, $hw_protocol, $hook) = unpack("L>S>C", $nla_data);
            print " - packet_id: $packet_id\n";
            print " - hw_protocol: $hw_protocol\n";
            print " - hook: $hook\n";
            push @pkt_ids, $packet_id;
        } elsif($nla_type == $NFQA_HWADDR){
            my ($hw_addrlen, $pad) = unpack("S>S", $nla_data);
            my $hw_addr = substr($nla_data, 0, $hw_addrlen, '');
            print " - hw_addrlen: $hw_addrlen\n";
            print " - hw_addr: ".to_hex($hw_addr)."\n";
        } elsif($nla_type == $NFQA_TIMESTAMP){
            my ($sec, $usec) = unpack("Q>Q>", $nla_data);
            print " - sec: $sec\n";
            print " - usec: $usec\n";
        } elsif($nla_type == $NFQA_IFINDEX_INDEV){
            my $ifindex = unpack("L>", $nla_data);
            print " - ifindex: $ifindex\n";
        } elsif($nla_type == $NFQA_IFINDEX_OUTDEV){
            my $ifindex = unpack("L>", $nla_data);
            print " - ifindex: $ifindex\n";
        } elsif($nla_type == $NFQA_IFINDEX_PHYSINDEV){
            my $ifindex = unpack("L>", $nla_data);
            print " - ifindex: $ifindex\n";
        } elsif($nla_type == $NFQA_IFINDEX_PHYSOUTDEV){
            my $ifindex = unpack("L>", $nla_data);
            print " - ifindex: $ifindex\n";
        } elsif($nla_type == $NFQA_PAYLOAD){
            print " - payload[hex]: ".to_hex($nla_data)."\n";
            my $ip_version = unpack("C", $nla_data);
            my ($iphdr, $tcphdr, $payload);
            if(($ip_version & 0xf0) == 0x40){
                ($iphdr, $tcphdr, $payload) = unpack("a20a32a*", $nla_data);
                print " - iphdr[hex]: ".to_hex($iphdr)."\n";
                # let's parse the ipv4 header
                my ($ip_version_ihl, $ip_dscp_ecn, $ip_tot_len, $ip_id, $ip_flags_fragment_offset, $ip_ttl, $ip_protocol, $ip_check, $ip_saddr, $ip_daddr) = unpack("CCS>S>S>CCS>a4a4", $iphdr);
                my $ip_version = $ip_version_ihl >> 4;
                my $ip_ihl = $ip_version_ihl & 0x0f;
                print " - ip_version: $ip_version\n";
                print " - ip_ihl: $ip_ihl\n";
                print " - ip_dscp_ecn: $ip_dscp_ecn\n";
                print " - ip_tot_len: $ip_tot_len\n";
                print " - ip_id: $ip_id\n";
                print " - ip_flags_fragment_offset: $ip_flags_fragment_offset\n";
                print " - ip_ttl: $ip_ttl\n";
                print " - ip_protocol: $ip_protocol\n";
                print " - ip_check: $ip_check\n";
                print " - ip_saddr: ".inet_ntoa($ip_saddr)."\n";
                print " - ip_daddr: ".inet_ntoa($ip_daddr)."\n";
            } elsif(($ip_version & 0xf0) == 0x60){
                ($iphdr, $tcphdr, $payload) = unpack("a40a32a*", $nla_data);
                print " - ipv6hdr[hex]: ".to_hex($iphdr)."\n";
                # let's parse the ipv6 header
                my ($ip6_version, $ip6_traffic_class, $ip6_flow_label, $ip6_payload_len, $ip6_next_header, $ip6_hop_limit, $ip6_src, $ip6_dst) = unpack("CCS>S>CCa16a16", $iphdr);
                $ip6_version &= 0xff;
                print " - ip6_version: $ip6_version\n";
                print " - ip6_traffic_class: $ip6_traffic_class\n";
                print " - ip6_flow_label: $ip6_flow_label\n";
                print " - ip6_payload_len: $ip6_payload_len\n";
                print " - ip6_next_header: $ip6_next_header\n";
                print " - ip6_hop_limit: $ip6_hop_limit\n";
                print " - ip6_src: ".inet_ntop(AF_INET6, $ip6_src)."\n";
                print " - ip6_dst: ".inet_ntop(AF_INET6, $ip6_dst)."\n";
            } else {
                print " - unknown ip_version: $ip_version:".($ip_version & 0xff)."\n";
            }
            print " - tcphdr[hex]: ".to_hex($tcphdr)."\n";
            print " - payload[hex]: ".to_hex($payload)."\n";
            print " - payload[raw]: ".$payload."\n";
        } elsif($nla_type == $NFQA_CT){
            print " - conntrack[hex]: ".to_hex($nla_data)."\n";
        } elsif($nla_type == $NFQA_CT_INFO){
            print " - conntrack_info[hex]: ".to_hex($nla_data)."\n";
        } elsif($nla_type == $NFQA_MARK){
            my $mark = unpack("L>", $nla_data);
            print " - mark: $mark\n";
        } elsif($nla_type == $NFQA_VLAN){
            my ($vlan_tci, $vlan_proto) = unpack("S>S>", $nla_data);
            print " - vlan_tci: $vlan_tci\n";
            print " - vlan_proto: $vlan_proto\n";
        } elsif($nla_type == $NFQA_L2HDR){
            print " - l2hdr[hex]: ".to_hex($nla_data)."\n";
        } elsif($nla_type == $NFQA_EXP){
            print " - exp[hex]: ".to_hex($nla_data)."\n";
        } elsif($nla_type == $NFQA_UID){
            my $uid = unpack("L>", $nla_data);
            print " - uid: $uid\n";
        } elsif($nla_type == $NFQA_GID){
            my $gid = unpack("L>", $nla_data);
            print " - gid: $gid\n";
        } elsif($nla_type == $NFQA_SECCTX){
            print " - secctx[hex]: ".to_hex($nla_data)."\n";
        } elsif($nla_type == $NFQA_SKB_INFO){
            print " - skb_info[hex]: ".to_hex($nla_data)."\n";
        } else {
            print " - unknown $nla_type\n";
        }
        substr($nl_attrs, 0, $nr_pad, '');
    }
    return \@pkt_ids;
}

sub to_hex {
    my $b = \$_[0];
    return join("",map {sprintf("%02x", ord $_)} split '', $$b//"");
}
