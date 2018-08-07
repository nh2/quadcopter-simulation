#!/usr/bin/env python3

# Simply pasted `avr-objdump --disassemble *.elf` output
# into the stdin of this program, e.g.
#    00000384 <micros>:
#     384:   60 91 7e 01     lds     r22, 0x017E
#     388:   70 91 7f 01     lds     r23, 0x017F
#     38c:   cb 01           movw    r24, r22
#     38e:   96 95           lsr     r25
#     390:   87 95           ror     r24
# prints "7".

import fileinput

def main():
  cycles_sum = 0
  for line in fileinput.input():
    print(line)
    if len(line) > 24:
      s = line[24:].split()
      if len(s) > 0:
        instr = s[0]
        cycles_sum += {
          'adc': 1,
          'add': 1,
          'andi': 1,
          'eor': 1,
          'in': 1,
          'lds': 2,
          'lsr': 1,
          'movw': 1,
          'or': 1,
          'ret': 4,
          'ror': 1,
          'sbci': 1,
          'subi': 1,
        }[instr]
  print(cycles_sum)

if __name__ == '__main__':
  main()
