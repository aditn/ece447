In our design, we left the majority the handling of control signals to our 
mips_decoder module, such as enabling registers, selecting inputs and 
outputs for muxes, and selecting operations for our own modules to perform.
For wires with multiple different inputs, we used muxes that were controlled
by the mips_decoder. For load and store operations, we created special loader
and storer modules to handle storing and loading bytes and halfwords. Also,
our ALU handled branch conditions. For our HI and LO registers, we simply used
normal registers that were enabled by the decoder.

The critical path of our synthesized implementation is 8.3 ns.

We spent approximately 1 hour planning the design, 18 hours capturing the 
design, 6 hours testing the design, and 4 hours debugging.
