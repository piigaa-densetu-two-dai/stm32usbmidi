MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Enforce emmition of the vector table. */
EXTERN(vector_table)

/* Define the entry point of the output file. */
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
	.text : {
		*(.vectors)	/* Vector table */
		*(.romtext*)	/* Program code */
	} > rom

	.ram_vectors (NOLOAD) : {
		*(.ram_vectors)
	} > ram

	/* ram, but not cleared on reset, eg boot/app comms */
	.noinit (NOLOAD) : {
		*(.noinit*)
	} > ram

	.data : {
		. = ALIGN(4);
		_data = .;
		*(.text*)	/* "text" functions to run in ram */
		*(.rodata*)	/* Read-only data */
		*(.data*)	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} > ram AT > rom
	_data_loadaddr = LOADADDR(.data);

	.bss : {
		*(.bss*)	/* Read-write zero initialized data */
		. = ALIGN(4);
		_ebss = .;
	} > ram
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
