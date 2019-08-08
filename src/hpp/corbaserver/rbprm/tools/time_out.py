from __future__ import print_function
import signal

# Register an handler for the timeout
def handler_factory(name, t):
	def handler(signum, frame):
		raise Exception("end of time allowed for method " + name + ": " + str(t))
	return handler

def run_time_out(method, t, *args):
	hd = handler_factory(method.__name__, t)
	signal.signal(signal.SIGALRM, hd)
	signal.alarm(t)
	method(*args)
	signal.alarm(0)
	


if __name__ == '__main__':
	def loop(dt1, dt2):
		import time
		for i in range(dt1 + dt2):
			print("sec")
			time.sleep(1)
        
	try:
		run_time_out(loop, 3, 3, 2)
	except Exception as exc: 
		print(exc)
		print("exception caught, ok")
		
	try:
		run_time_out(loop, 6, 3, 2)
	except Exception as exc: 
		print(exc)
		print("exception should NOT becaught")
