DTS=bone_eqep0.dts bone_eqep1.dts bone_eqep2.dts bone_eqep2alt.dts

DTBs=$(patsubst %.dts,%.dtbo,$(DTS))

%.dtbo: %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

all: $(DTBs)

clean-local:
	rm -f  $(DTBs)

