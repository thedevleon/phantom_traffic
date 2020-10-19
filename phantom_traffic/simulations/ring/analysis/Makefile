# tool for indexing vec files
SCAVETOOL = scavetool
# scripts location
SCRIPTDIR = .
# results location
RESDIR = ../results
# script for merging
MERGESCRIPT = $(SCRIPTDIR)/merge.R

# match all .vec files for the EngineTest config
ENGINETEST      = $(wildcard $(RESDIR)/EngineTest*.vec)
# change suffix from .vec to .Rdata and add the eng prefix
ENGINETEST_DATA = $(ENGINETEST:$(RESDIR)/%.vec=$(RESDIR)/eng.%.Rdata)

# vector index files and Rdata files
VCI = $(VECTOR:%.vec=%.vci)
RDATA = $(VECTOR:%.vec=%.Rdata)
CSV = $(VECTOR:%.vec=%.csv)

# all make targets
all: EngineTest.Rdata 

# to make EngineTest.Rdata we need to merge all files starting with eng.EngineTest
# before this, check that all ENGINETEST_DATA files have been processed
$(RESDIR)/EngineTest.Rdata: $(ENGINETEST_DATA)
	Rscript $(MERGESCRIPT) $(RESDIR)/ eng.EngineTest $(notdir $@) map-config default Rdata
EngineTest.Rdata: $(RESDIR)/EngineTest.Rdata

# to make all eng.*.Rdata files we need to run the generic parser
eng.%.Rdata: %.vec %.vci
	Rscript generic-parser.R $< map-config default eng Rdata

# if vec files are not indexed, index them
%.vci : %.vec
	$(SCAVETOOL) index $<

# helper to print variable values. e.g.: make print-DELAY_DATA
print-%:
	@echo '$*=$($*)'

# every intermediate file is kept instead of being automatically deleted.
# .vci files are detected as intermediate and thus cancelled when the make
# command terminates. however, such files can still be needed by other
# targets, so keep them instead of re-doing indexing
.SECONDARY :
