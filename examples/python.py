import genomix

handle = genomix.connect("localhost:8080")
handle.rpath("/home/franklinselva/dev/work/drone-experiment/lib/genom/pocolibs/plugins")

d435 = handle.load("d435")
foxglove = handle.load("FoxgloveStudio")
ct = handle.load("ColorTracker")

d435.connect