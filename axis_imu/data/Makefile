
PCAPFILE=motionlogger_udp_0_5_0_doubles_network_byte_order.pcap

terra8:
	tcprewrite --infile=$(PCAPFILE) --outfile=changed.pcap --dstipmap=0.0.0.0/0:192.168.7.160/32 --fixcsum

laptop:
	tcprewrite --infile=$(PCAPFILE) --outfile=changed.pcap --dstipmap=0.0.0.0/0:192.168.1.136/32 --fixcsum

replay-terra8:
	udpreplay -i ens1 -c 1000 changed.pcap

replay:
	udpreplay -i wlp2s0 -c 1000 changed.pcap
