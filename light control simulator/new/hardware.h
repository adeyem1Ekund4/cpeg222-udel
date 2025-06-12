#ifndef HARDWARE_H
#define HARDWARE_H

void initialize_ports();
void initialize_output_states();

#define SW6 PORTBbits.RB10
#define SW7 PORTBbits.RB9

int BtnR_read_raw();

#endif
