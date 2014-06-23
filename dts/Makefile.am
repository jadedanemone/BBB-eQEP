DTS=fast_eqep0.dts fast_eqep1.dts fast_eqep2.dts fast_eqep2alt.dts

DTBOs=$(patsubst %.dts,%-00A0.dtbo,$(DTS))

%-00A0.dtbo: %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

all: $(DTBOs)

clean-local:
	rm -f  $(DTBOs)

install:
	install -m 0644 $(DTBOs) /lib/firmware

