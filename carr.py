
import sys

name = sys.argv[1]

f = open(f'{name}.txt', 'r')
i = 0
for l in f:
    x = l.split()
    x = x[2:-1] if name == 'x' else x[2:3]
    for xi in range(len(x)):
        print(f'acadoVariables.{name}[{i+xi}] = {x[xi]};')
    i += len(x)
f.close()