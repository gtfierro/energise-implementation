import struct 

def hex2float(s):
	if(s[0:2]=='0x'):
		s = s[2:len(s)]
	bins = ''.join(chr(int(s[x:x+2], 16)) for x in range(0, len(s), 2))
	return struct.unpack('>f', bins)[0]

def float2int(num):
	return struct.unpack('=i',struct.pack('=f',num))[0];
def concatData(data):
	tVal = 0
	upper = True
	for reg in data:
		if upper:
			tVal = ((reg & 0xFFFF) << 16)
			upper = False
		else:
			tVal = tVal | (reg & 0xFFFF)
			upper = True
	return tVal

'''
Converting Numbers to 16bit data array's
'''
def int16_to_data(num):
	return struct.unpack('=H',struct.pack('=h',num))[0];
def uint16_to_data(num):
	return struct.unpack('=H',struct.pack('=H',num&0xFFFF))[0];
def uint32_to_data(num):
	data = [0,0]
	data[0] = struct.unpack('=H',struct.pack('=H',(num>>16)&0xffff))[0];
	data[1] = struct.unpack('=H',struct.pack('=H',num&0xffff))[0];
	return data;
def int32_to_data(num):
	data = [0,0]
	data[0] = struct.unpack('=H',struct.pack('=H',(num>>16)&0xffff))[0];
	data[1] = struct.unpack('=H',struct.pack('=H',num&0xffff))[0];
	return data;
def float32_to_data(num):
	intNum = float2int(num)
	data = [0,0]
	data[0] = (intNum >>16) & 0xFFFF
	data[1] = intNum & 0xFFFF
	return data
def float32_to_data_inv(num):
	intNum = float2int(num)
	data = [0,0]
	data[0] = intNum & 0xFFFF
	data[1] = (intNum >>16) & 0xFFFF
	return data

'''
Converting Data array's to Numbers
'''
def data_to_int16(data):
	return struct.unpack("h",struct.pack("H",data[0]))[0]
def data_to_uint16(data):
	return data[0]
def data_to_uint32(data):
	return concatData(data)
def data_to_int32(data):
	return struct.unpack('=i',struct.pack('=I',concatData(data)))[0];
def data_to_float32(data):
	return hex2float(hex(concatData(data)))
def data_to_float32_inv(data):
	temp = []
	temp.append(data[1])
	temp.append(data[0])
	return struct.unpack('=f',struct.pack('=i',concatData(temp)))[0]