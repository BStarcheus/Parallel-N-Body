# Parallel-N-Body

bodies.txt format: name,color,mass,radius,pos.x,pos.y,vel.x,vel.y

### Run
Run on OSC so you don't need to install matplotlib on python2.7, cuda, etc.

Sequential Version:
```
$ g++ seq.cpp -I/usr/include/python2.7 -lpython2.7
$ ./a.out bodies.txt
```

Parallel Version:
```
$ module load cuda
$ nvcc parallel.cu
$ ./a.out bodies.txt
```