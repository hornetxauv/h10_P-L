to = 303
with open('output.txt', 'w') as file:
    for i in range(0,round(to/10)+1):
        tenend = (i+1)*10
        end = to if to < tenend else tenend
        file.write(f'{i*10+1}-{end}\n')