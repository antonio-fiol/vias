#
# Main Makefile. This is basically the same as a component makefile.
#
COMPONENT_EMBED_TXTFILES :=  ca_cert.pem build_time.txt

$(COMPONENT_PATH)/build_time.txt:
	echo DATE
	echo -n `date +%Y%m%d%H%M%S` > $@

