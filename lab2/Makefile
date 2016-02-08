#
# ECE 18-447 Makefile.
#
# - History:
#   - 2011: Joshua Wise
#   - 2013: Yoongu Kim
#   - 2016: Zhipeng Zhao
#


###################################################################
### Setup
###################################################################

TIMESTAMP := outputs/$(shell date +%m%d-%H%M%S)
OUTPUT ?= $(TIMESTAMP)

INPUT ?= 447inputs/addiu.s
REGDUMP := $(INPUT:.s=.reg)

BATCH ?= 447inputs/addiu.s

SYNTHDIR = dc
SYNTH = lab
TB = testbench

###################################################################
### Constants
###################################################################

# text attributes: normal, bold, underline
n=\e[0m
b=\e[1m
u=\e[4m

# bold+green
g=\e[1;32m

# bold+red
r=\e[1;31m

# debug message
m=$g18-477 Makefile: $n


###################################################################
### Help Screen
###################################################################

default:
	@echo -e "$mHELP SCREEN"
	@echo -e ""
	@echo -e "$bUSAGE:$n"
	@echo -e "\tmake $uTARGET$n [$uVARIABLE$n]..."
	@echo -e ""
	@echo -e ""
	@echo -e "$bTARGET:$n"
	@echo -e "\t$bsim$n"
	@echo -e "\t    compiles Verilog found in $usrc$n directory;"
	@echo -e "\t    simulates Verilog using $bINPUT$n as input;"
	@echo -e "\t    places all outputs (logs, waveforms, regdump, etc.) in $bOUTPUT$n"
	@echo -e ""
	@echo -e "\t$bverify$n"
	@echo -e "\t    compiles Verilog found in $usrc$n directory;"
	@echo -e "\t    simulates Verilog using $bINPUT$n as input;"
	@echo -e "\t    places all outputs (logs, waveforms, regdump, etc.) in $bOUTPUT$n"
	@echo -e "\t    verifies the simulation register-dump against the reference"
	@echo -e ""
	@echo -e "\t$bbatch-verify$n"
	@echo -e "\t    similar to $bverify$n, but uses $bBATCH$n as input"
	@echo -e ""
	@echo -e "\t$bsynth$n"
	@echo -e "\t    compiles Verilog found in $usrc$n directory;"
	@echo -e "\t    synthesizes Verilog"
	@echo -e "\t    places all outputs (reports, schematics, etc.) in $bOUTPUT$n"
	@echo -e ""
	@echo -e "$bVARIABLE:$n"
	@echo -e "\t$bINPUT$n=$uASSEMBLY$n"
	@echo -e "\t    for $bsim$n and $bverify$n, specifies an input MIPS assemply file;"
	@echo -e "\t    if unspecified, defaults to $u447inputs/addiu.s$n"
	@echo -e ""
	@echo -e "\t$bBATCH$n=$uASSEMBLY FILES$n"
	@echo -e "\t    for $bbatch-verify$n, specifies one or more input MIPS assemply files;"
	@echo -e "\t    if unspecified, defaults to $u447inputs/addiu.s$n"
	@echo -e ""
	@echo -e "\t$bOUTPUT$n=$uDIR$n"
	@echo -e "\t    for all targets, specifies the output directory;"
	@echo -e "\t    if unspecified, defaults to $uoutputs/current-timestamp$n;"
	@echo -e ""
	@echo -e ""
	@echo -e "$bEXAMPLES:$n"
	@echo -e "\tmake sim"
	@echo -e "\tmake sim INPUT=inputs/mytest.s OUTPUT=outputs/myoutput"
	@echo -e "\tmake verify"
	@echo -e "\tmake verify INPUT=inputs/mytest.s OUTPUT=outputs/myoutput"
	@echo -e "\tmake batch-verify BATCH=447inputs/*.s"
	@echo -e "\tmake batch-verify BATCH=447inputs/*.s OUTPUT=outputs/myoutput"
	@echo -e "\tmake synth"
	@echo -e "\tmake synth OUTPUT=outputs/myoutput"
	@echo -e ""
	@echo -e ""
	@echo -e "(END)"
	@exit 0


###################################################################
### Compile Verilog
###################################################################

compile:
	@echo -e "$mCopying Verilog into $(OUTPUT)..."
	@mkdir -p $(OUTPUT)/sim/src
	@cp `find src 447src -iname '*.sv' -o -iname '*.v' -o -iname '*.vh'`	$(OUTPUT)/sim/src
	@echo -e "$mCompiling Verilog..."
	cd $(OUTPUT)/sim; ncvlog -SV -linedebug -messages -incdir src src/*.v $(shell ls src/*.sv)
	@if grep '*W' $(OUTPUT)/sim/ncvlog.log >/dev/null; \
		then echo -e '$m$rCompiler log has warnings!$n'; fi
	@echo -e "$mElaborating Verilog..."
	cd $(OUTPUT)/sim; ncelab +access+rwc -messages worklib.$(TB)
	@if grep '*W' $(OUTPUT)/sim/ncelab.log >/dev/null; \
		then echo -e '$m$rElaborator log has warnings!$n'; fi


###################################################################
### Assemble MIPS Assembly Input
###################################################################

assemble:
	@echo -e "$mCopying $(INPUT) into $(OUTPUT)..."
	@mkdir -p $(OUTPUT)/sim
	@([ -f $(INPUT) ] && cp $(INPUT) $(OUTPUT)/sim) || \
		(echo -e "$mERROR -- test input not found: $(INPUT)"; exit 1)
	@echo -e "$mAssemblying input in $(OUTPUT)..."
	spim447 -notrap -vasm $(INPUT) $(OUTPUT)/sim/mem;


###################################################################
### Simulate Verilog
###################################################################

sim: compile assemble
	@echo -e "$mCopying NCSim configuration into $(OUTPUT)..."
	@cp ncsim/* $(OUTPUT)/sim
	@cp 447ncsim/* $(OUTPUT)/sim
	@echo -e "$mSimulating Verilog in $(OUTPUT)..."
	cd $(OUTPUT)/sim; ncsim worklib.$(TB):module -input 447ncsim.tcl -input ncsim.tcl
	@if grep '*W' $(OUTPUT)/sim/ncsim.log >/dev/null; \
		then echo -e '$m$rSimulator log has warnings!$n'; fi
	@echo -e "$mSimulation of $(INPUT) has completed in $(OUTPUT)."
	@echo -e "$mTo view waveforms, execute the following command: $bsimvision $(OUTPUT)/sim/waveforms.shm$n"


###################################################################
### Verify Simulation
###################################################################

verify: sim
	@([ -f $(REGDUMP) ] && cp $(REGDUMP) $(OUTPUT)/sim) || \
		(echo -e "$mERROR -- reference register-dump not found: $(REGDUMP)"; exit 1)
	447util/tester.pl $(OUTPUT)/sim/$(notdir $(REGDUMP)) $(OUTPUT)/sim/regdump.txt
	@echo -e "$mVerification of $(INPUT) has completed in $(OUTPUT)."
	@echo -e "$mTo view the simulation register-dump, execute the following command: $bcat $(OUTPUT)/sim/regdump.txt$n"

batch-verify:
	for i in $(BATCH); do \
		make verify INPUT=$$i OUTPUT=$(OUTPUT); \
	done


###################################################################
### Synthesize Verilog
###################################################################

synth:
	@mkdir -p $(OUTPUT)/synth/src
	@mkdir -p $(OUTPUT)/synth/447src
	@cp `find src -iname '*.v' -o -iname '*.sv' -o -iname '*.vh'` $(OUTPUT)/synth/src
	@cp `find 447src -iname '*.v' -o -iname '*.sv' -o -iname '*.vh'` $(OUTPUT)/synth/447src
	@cp $(SYNTHDIR)/$(SYNTH).dc $(OUTPUT)/synth
	@cd $(OUTPUT)/synth; dc_shell-xg-t -f $(SYNTH).dc
	@echo -e "$mSynthesis has completed in $(OUTPUT)."
	@echo -e "Timing results can be found at $(OUTPUT)/synth/timing_mips_core.rpt"
