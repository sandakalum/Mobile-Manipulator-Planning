#!/usr/bin/python

fd = file('refman.tex', 'r')
fd2 = file('filelist.tex', 'w')

line = fd.readline()
while line.find('Module Index') < 0:
    line = fd.readline()

fd2.write(line)
line = fd.readline()

while line.find('printindex') < 0:
    fd2.write(line)
    line = fd.readline()

fd.close()
fd2.close()
