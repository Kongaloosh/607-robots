from multiprocessing import Process, Queue

def f(al):
    al.put([i for i in range(20)])

if __name__ == '__main__':
    manager = Queue()

    al = Queue()

    p = [Process(target=f, args=(al,)) for i in range(10)]
    for i in p:
        i.start()

    for i in p:
        i.join()

    for i in p:
        print(al.get())