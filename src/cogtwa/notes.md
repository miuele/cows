## implementation notes

### OD read/write hooks
Currently `OD_extension_init` is called whenever OD notification is set up. This should work fine in the current implementation of CANopenNode, but this may change in future releases of the library. An alternative approach is to call `OD_extension_init` on all possible entries at configuration phase, but this comes with large memory footprint and possible performance penalty. Therefore current decision is to keep it as it is.
