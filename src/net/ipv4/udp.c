/**
 * \file udp.c
 *
 * \date 26.03.2009
 * \author sikmir
 * \details The User Datagram Protocol (UDP).
 */
#include "string.h"
#include "common.h"
#include "net/skbuff.h"
#include "net/net.h"
#include "net/eth.h"
#include "net/if_device.h"
#include "net/net_pack_manager.h"
#include "net/udp.h"
#include "net/ip.h"
#include "net/icmp.h"
#include "net/socket.h"
#include "net/if_ether.h"
#include "lib/inet/netinet/in.h"

int udp_rcv(sk_buff_t *pack) {
	LOG_WARN("udp packet received\n");
	return udpsock_push(pack);
}

static SOCK_INFO *__udp_lookup(unsigned char saddr[4], unsigned short source,
				unsigned char daddr[4], unsigned short dest) {
	int i;
	struct udp_sock *usk;
	for(i=0; i< MAX_SOCK_NUM; i++) {
		usk = sks[i].sk;
		if(dest == usk->inet.sport &&
		   ((0 == memcmp(daddr, usk->inet.saddr, 4)) ||
		   (0 == inet_addr(usk->inet.saddr)))) {
			return &sks[i];
		}
	}
	return NULL;
}

static int udp_queue_rcv_pack(SOCK_INFO *sk, sk_buff_t *pack) {
	if(sk->new_pack == 0) {
		sk->queue = skb_copy(pack, 0);
		sk->new_pack = 1;
		sk->sk->inet.dport = pack->h.uh->source;
		memcpy(sk->sk->inet.daddr, pack->nh.iph->saddr, sizeof(pack->nh.iph->saddr));
		LOG_DEBUG("packet pushed\n");
		return 0;
	}
	LOG_DEBUG("queue is busy\n");
	return -1;
}

int udpsock_push(sk_buff_t *pack) {
        LOG_DEBUG("push packet to udp socket\n");
        int i;
        SOCK_INFO *sk;
        udphdr *uh = pack->h.uh;
        unsigned short ulen = ntohs(uh->len);
        iphdr_t *iph = pack->nh.iph;
        unsigned char saddr[4];
        unsigned char daddr[4];
        memcpy(saddr, pack->nh.iph->saddr, 4);
        memcpy(daddr, pack->nh.iph->daddr, 4);
        /**
         *  Validate the packet.
         */
        if (ulen > pack->len) {
    		return -1;
	}
	unsigned short tmp = uh->check;
	uh->check          = 0;
	if ( tmp !=  calc_checksumm(uh, UDP_HEADER_SIZE)) {
		LOG_ERROR("bad udp checksum\n");
		return -1;
	}
        sk = __udp_lookup(saddr, uh->source, daddr, uh->dest);
	if (sk != NULL) {
		return udp_queue_rcv_pack(sk, pack);
	}
    	LOG_DEBUG("destination socket not found\n");
    	icmp_send(pack, ICMP_DEST_UNREACH, ICMP_PORT_UNREACH);
    	return -1;
}

int udp_init() {
	return 0;
}

static int rebuild_udp_header(sk_buff_t *pack, unsigned short source, unsigned short dest) {
	LOG_DEBUG("rebuild udp header\n");
	udphdr *hdr = pack->h.uh;
	hdr->source = source;
	hdr->dest   = dest;
	hdr->len    = UDP_HEADER_SIZE;
	hdr->check  = 0;
	hdr->check  = calc_checksumm(hdr, UDP_HEADER_SIZE);
	return 0;
}

static void rebuild_udp_packet(sk_buff_t *pack, struct udp_sock *sk, void *ifdev, const void *buf, int len) {
	LOG_DEBUG("rebuild_udp_packet\n");
	if( pack == NULL ||
	    ifdev == NULL ||
	    sk ==NULL) {
		return;
	}
	//TODO UDP get net dev
#if 0
	pack->ifdev = ifdev;
	pack->netdev = inet_dev_get_netdevice(ifdev);
#endif
	pack->len = UDP_HEADER_SIZE;

	pack->h.raw = pack->data + ETH_HEADER_SIZE + IP_HEADER_SIZE;
	memset(pack->h.raw, 0, UDP_HEADER_SIZE);
	rebuild_udp_header(pack, sk->inet.sport, sk->inet.dport);
//	memcpy(pack->h.uh->data, buf, len);
}

int udp_trans(struct udp_sock *sk, void *ifdev, const void *buf, int len) {
	LOG_DEBUG("udp_trans\n");
	sk_buff_t *pack;
        pack = alloc_skb(len, 0);
        if( pack == NULL) {
    		return -1;
        }
	rebuild_udp_packet(pack, sk, ifdev, buf, len);
	return ip_send_packet(&sk->inet, pack);
}
