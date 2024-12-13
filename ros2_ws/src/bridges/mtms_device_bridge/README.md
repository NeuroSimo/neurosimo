# Environment variables
If running without docker, ensure that the following environment variables are set:

- `BITFILE` is the name of the bitfile, for example `NiFpga_mTMS_Tubingen_0.4.1.lvbitx`.
- `BITFILE_DIRECTORY` is the path to the directory in which the bitfile is, e.g., `/home/mtms/workspace/mtms/bitfiles/`.
- `BITFILE_SIGNATURE` is the signature of the file, e.g., `B5B3B2DAFB64E53F87C9C0EF36D1D936`. The signatures are stored as text files
in the same directory as the corresponding bitfiles.
- `RESOURCE` is the name of the FPGA resource, e.g., `PXI1Slot3`.
