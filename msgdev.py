import zmq
import threading
import time
import sys
import array
import struct

__all__ = [
	'MsgDeviceError',
	'PeriodTimer',
	'MsgDevice'
]


class MsgDeviceError(Exception):
	pass


if sys.platform == 'linux2':
	# time() is not perfect wall-time clock, but quicker than ctype clock_gettime()
	monoclock = time.time
else:
	monoclock = time.clock


class PeriodTimer(object):
	def __init__(self, interval):
		self.interval = interval
		self.i = 0

	def start(self):
		self.start = monoclock()

	def __enter__(self):
		self.i += 1

	def __exit__(self, exc_type, exc_val, exc_tb):
		if exc_type is not None:
			return
		end = self.start + self.interval * self.i
		now = monoclock()
		if now >= end:
			return
		else:
			time.sleep(end - now)


class MsgDevice(object):
	def __init__(self):
		if sys.byteorder == 'little':
			self.a2b = self.a2b_little
			self.b2a = self.b2a_little
		else:
			self.a2b = self.a2b_big
			self.b2a = self.b2a_big

	def a2b_little(self, arr):
		return array.array('d', arr).tostring()

	def b2a_little(self, bytes):
		return array.array('d', bytes)

	def a2b_big(self, arr):
		a = array.array('d', arr)
		a.byteswap()
		return a.tostring()

	def b2a_big(self, bytes):
		a = array.array('d', bytes)
		a.byteswap()
		return a

	def open(self):
		self.ctx = zmq.Context()
		self._bind_pair()
		self._init_thread()

	def _init_thread(self):
		self.sub_dict_lock = threading.Lock()
		self.pub_dict_lock = threading.Lock()
		self.sub_dict = {}
		self.pub_dict = {}
		self.thread = MsgThread(self.ctx, self)
		self.thread.start()
		self.alive_ck = 0
		self._send_cmd(('ping',))

	def _bind_pair(self):
		import uuid
		self.pair_url = "inproc://" + uuid.uuid1().hex
		self.pair = self.ctx.socket(zmq.PAIR)
		self.pair.bind(self.pair_url)

	def _recv_reply(self, block=True):
		if self.pair.poll(10000, zmq.POLLIN):
			msgs = self.pair.recv_pyobj()
		else:
			raise MsgDeviceError("msgthread inresposive")

		if msgs[0] == 'OK':
			return
		elif msgs[0] == 'ERROR':
			raise msgs[1]

	def _send_cmd(self, msgs, check_reply=True):
		blocked = False
		try:
			self.pair.send_pyobj(msgs, zmq.NOBLOCK)
		except zmq.ZMQError as e:
			if e.errno == zmq.EAGAIN:
				blocked = True
			else:
				raise
		if blocked:
			if self.thread.is_alive() \
				and self.pair.poll(1000, zmq.POLLOUT):
				self.pair.send_pyobj(msgs, zmq.NOBLOCK)
			else:
				raise MsgDeviceError("msgthread inresposive")

		if check_reply:
			self._recv_reply()

	def set_repub_interval(self, interval):
		if 0.1 <= interval <= 10.0:
			self._send_cmd(('set_repub_interval', str(interval)))
		else:
			raise MsgDeviceError("repub interval must be >=0.1s or <=10.0s, got " + str(interval))

	def pub_connect(self, endpoint):
		self._send_cmd(('pub_connect', endpoint))

	def pub_bind(self, endpoint):
		self._send_cmd(('pub_bind', endpoint))

	def sub_connect(self, endpoint):
		self._send_cmd(('sub_connect', endpoint))

	def sub_bind(self, endpoint):
		self._send_cmd(('sub_bind', endpoint))

	def sub_add_url(self, url, default_values=(0,)):
		url = url + '$'
		t = type(default_values)
		if t == int or t == float:
			default_values = (default_values,)
		value = self.a2b(default_values)
		with self.sub_dict_lock:
			self.sub_dict[url] = value

		self._send_cmd(('add_sub_url', url))

	def pub_set1(self, url, value):
		self.pub_set(url, (value,))

	def pub_set(self, url, value):
		self.alive_ck += 1
		if self.alive_ck % 100 == 1 and not self.thread.isAlive():
			self._recv_reply()
			return

		url = url + '$'
		new_pub = False

		with self.pub_dict_lock:
			if len(self.pub_dict) == 0:
				new_pub = True
			self.pub_dict[url] = value

		if new_pub:
			self._send_cmd(('new_pub',), False)

	def sub_get1(self, url):
		v = self._sub_get_bytes(url)
		if len(v) != 8:
			raise MsgDeviceError('not a single value sub: ' + url)
		return self.b2a(v)[0]

	def sub_get(self, url):
		return self.b2a(self._sub_get_bytes(url))

	def _sub_get_bytes(self, url):
		self.alive_ck += 1
		if self.alive_ck % 100 == 1 and not self.thread.isAlive():
			self._recv_reply()
			return

		url = url + '$'
		notfound = False

		with self.sub_dict_lock:
			if url in self.sub_dict:
				value = self.sub_dict[url]
			else:
				notfound = True

		if notfound:
			raise MsgDeviceError('unknown sub url: ' + url)

		return value

	def close(self):
		self.stop_thread()
		self.close_pair()
		self.close_ctx()

	def stop_thread(self):
		if hasattr(self, 'thread'):
			if self.thread.isAlive():
				try:
					self._send_cmd(('stop',))
				except:
					pass
				self.thread.join()
			del self.thread

	def close_pair(self):
		if hasattr(self, 'pair'):
			self.pair.close()
			del self.pair

	def close_ctx(self):
		if hasattr(self, 'ctx'):
			self.ctx.destroy()
			del self.ctx

	# functions below is called in MsgThread, don't use it otherwise
	def set_sub_dict(self, subs):
		invalid_url = ""
		with self.sub_dict_lock:
			for url in subs:
				if url not in self.sub_dict \
					or len(self.sub_dict[url]) != len(subs[url]):
					invalid_url = url

				else:
					self.sub_dict[url] = subs[url]
		if invalid_url:
			raise MsgDeviceError(
				"subs recv invalid url: %s, len: %d"
				% (invalid_url, len(subs[invalid_url])))

	def get_pub_dict(self):
		with self.pub_dict_lock:
			pubs = self.pub_dict
			self.pub_dict = {}
		return pubs


class MsgThread(threading.Thread):
	def __init__(self, ctx, msg_device):
		threading.Thread.__init__(self)
		self.dev = msg_device
		self.ctx = ctx
		self.stmap = {}
		self.stmap[1] = struct.Struct('<d')

	def _conn_pair(self):
		self.pair = self.ctx.socket(zmq.PAIR)
		self.pair.connect(self.dev.pair_url)
		self.poller.register(self.pair, zmq.POLLIN)

	def _open_pub_sub(self):
		self.pub = self.ctx.socket(zmq.PUB)
		self.sub = self.ctx.socket(zmq.SUB)
		self.poller.register(self.sub, zmq.POLLIN)
		self.pub_registered = False

	def _thr_init(self):
		self.repub_interval = 1.0
		self.poll_time_ms = 1000
		self.next_repub_time = monoclock() + self.repub_interval
		self.has_new_pub = False
		self.stop_requested = False
		self.pub_dict = {}
		self.pending_pubs = {}
		self.wait_repubs = {}

		self.poller = zmq.Poller()
		self._conn_pair()
		self._open_pub_sub()

	def _thr_final(self):
		if hasattr(self, 'pub'):
			self.pub.close(1)
			del self.pub
		if hasattr(self, 'sub'):
			self.sub.close(1)
			del self.sub
		if hasattr(self, 'pair'):
			self.pair.close()
			del self.pair

	def _proc_pair_sock(self):
		try:
			while 1:
				msgs = self.pair.recv_pyobj(zmq.NOBLOCK)
				func_name = "_cmd_" + msgs[0]
				if hasattr(self, func_name):
					getattr(self, func_name)(msgs)
				else:
					raise MsgDeviceError("unknown cmd: " + msgs[0])
		except zmq.ZMQError as e:
			if e.errno != zmq.EAGAIN:
				raise

	def _reply_ok(self):
		self.pair.send_pyobj(("OK",))

	# start of pair command
	def _cmd_ping(self, msgs):
		self._reply_ok()

	def _cmd_set_repub_interval(self, msgs):
		self.repub_interval = float(msgs[1])
		self.next_repub_time = monoclock() + self.repub_interval
		self.poll_time_ms = int(self.repub_interval * 1000)
		self._reply_ok()

	def _cmd_pub_connect(self, msgs):
		self.pub.connect(msgs[1])
		self._reply_ok()

	def _cmd_sub_connect(self, msgs):
		self.sub.connect(msgs[1])
		self._reply_ok()

	def _cmd_pub_bind(self, msgs):
		self.pub.bind(msgs[1])
		self._reply_ok()

	def _cmd_sub_bind(self, msgs):
		self.sub.bind(msgs[1])
		self._reply_ok()

	def _cmd_add_sub_url(self, msgs):
		self.sub.setsockopt(zmq.SUBSCRIBE, msgs[1].encode('utf8'))
		self._reply_ok()

	def _cmd_new_pub(self, msgs):
		self.has_new_pub = True
		# no reply

	def _cmd_stop(self, msgs):
		self.stop_requested = True
		self._reply_ok()

	# end of pair command

	def _proc_sub_sock(self):
		subs = {}
		try:
			for i in range(100):
				msg = self.sub.recv_multipart(zmq.NOBLOCK)
				subs[msg[0].decode('utf8')] = msg[1]
		except zmq.ZMQError as e:
			if e.errno != zmq.EAGAIN:
				raise
		self.dev.set_sub_dict(subs)

	def _proc_pub_sock(self):
		self._send_pending_pubs()

	def _send_pending_pubs(self):
		if len(self.pending_pubs) > 0:
			try:
				for kv in list(self.pending_pubs.items()):
					self.pub.send_multipart(kv, zmq.NOBLOCK)
					del self.pending_pubs[kv[0]]
			except zmq.ZMQError as e:
				if e.errno != zmq.EAGAIN:
					raise

		if len(self.pending_pubs) == 0 and self.pub_registered:
			self.poller.unregister(self.pub)
			self.pub_registered = False
		elif len(self.pending_pubs) > 0 and not self.pub_registered:
			self.poller.register(self.pub, zmq.POLLOUT)
			self.pub_registered = True

	def _pack(self, v):
		l = len(v)
		if l in self.stmap:
			st = self.stmap[l]
		else:
			st = struct.Struct('<%dd' % l)
			self.stmap[l] = st
		return st.pack(*v)

	def _check_new_pub(self):
		if not self.has_new_pub:
			return

		pubs = self.dev.get_pub_dict()
		self.has_new_pub = False
		for k in pubs:
			kb = k.encode('utf8')
			v = self._pack(pubs[k])
			if kb not in self.pub_dict \
				or v != self.pub_dict[kb]:
				# sent pubs only if not same as cache version
				self.pub_dict[kb] = v
				self.pending_pubs[kb] = v
				self.wait_repubs.pop(kb, 0)

		self._send_pending_pubs()

	def _check_repub(self):
		now = monoclock()
		if now <= self.next_repub_time:
			self.poll_time_ms = int((self.next_repub_time - now) * 1000)
			if(self.poll_time_ms < 1):
				self.poll_time_ms = 1
		else:
			self.next_repub_time = now + self.repub_interval
			self.poll_time_ms = int(self.repub_interval * 1000)
			self.pending_pubs.update(self.wait_repubs)
			self.wait_repubs.update(self.pub_dict)
			self._send_pending_pubs()

	def _poll(self):
		event = self.poller.poll(self.poll_time_ms)
		socks = [x[0] for x in event]
		if self.pair in socks:
			self._proc_pair_sock()
		if self.sub in socks:
			self._proc_sub_sock()
		if self.pub in socks:
			self._proc_pub_sock()

	def run(self):
		try:
			self._thr_init()
			while 1:
				self._poll()
				if self.stop_requested:
					break
				self._check_new_pub()
				self._check_repub()
		except Exception as e:
			if hasattr(self, 'pair'):
				self.pair.send_pyobj(('ERROR', e))
			raise e
		finally:
			self._thr_final()
