MEMORY {
  DRAM_TOKENIZER  (rwx): ORIGIN = 0x90000000, LENGTH = 1024M
  DRAM_WEIGHTS    (rwx): ORIGIN = 0x9a000000, LENGTH = 2048M
}

SECTIONS {
  /* Contains the tokenizer at 0x90000000 */
  .ai.tokenizer (NOLOAD) : {
    . = ALIGN(4);
    *(.ai.tokenizer);
    /* ./models/tokenizer_name_here.bin */
  }> DRAM_TOKENIZER

  /* Contains the weights at 0x9a000000 */
  .ai.weights (NOLOAD) : {
    . = ALIGN(4);
    *(.ai.weights);
    /* ./models/weights_name_here.bin */
  }> DRAM_WEIGHTS
}
