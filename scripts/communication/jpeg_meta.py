import numpy
import json
import hashlib


APP0=[0xff,0xe0]
START=[0xff,0xd8]
COM_NUMBER=[0xff,0xfe]
SHA1_HEX_LENGTH = 40

def rolling_window(a, window):
    shape = a.shape[:-1] + (a.shape[-1] - window + 1, window)
    strides = a.strides + (a.strides[-1],)
    return numpy.lib.stride_tricks.as_strided(a, shape=shape, strides=strides)


def find_numpy(a, b):
    temp = rolling_window(a, len(b))
    result = numpy.where(numpy.all(temp == b, axis=1))
    return result[0] if result else None

def skipJfifMarkers(buf):
    offset=2
    for index in find_numpy(buf,APP0):
        count=int(buf[index+2])*256+int(buf[index+3])
        offset+=2+count
    return offset

def insertMetaData(buf,data):
    offset=skipJfifMarkers(buf)
    jsonstr=json.dumps(data)
    dataJson=numpy.frombuffer(jsonstr.encode('utf-8'),dtype=numpy.uint8)
    dataJsonLength=len(dataJson)
    dataMetaLength=dataJsonLength+SHA1_HEX_LENGTH
    ContentLength=dataMetaLength+2
    fullLength=dataMetaLength+4

    result=numpy.zeros((buf.shape[0]+fullLength),dtype=numpy.uint8)



    result[0:offset]=buf[0:offset]
    result[offset:offset+2]=COM_NUMBER[:]
    result[offset+2]=int(ContentLength/256)
    result[offset+3]=ContentLength%256
    result[offset+4+SHA1_HEX_LENGTH:offset+dataJsonLength+4+SHA1_HEX_LENGTH]=dataJson[0:dataJsonLength]
    result[offset+4+dataJsonLength+SHA1_HEX_LENGTH:]=buf[offset:]

    sha1 = hashlib.sha1()
    sha1.update(result)
    sha1_buf = numpy.frombuffer(sha1.hexdigest().encode('utf-8'), dtype=numpy.uint8)
    result[offset + 4:offset + 4 + SHA1_HEX_LENGTH] = sha1_buf[:]
    return result


if __name__=='__main__':
    import cv2
    import time

    img = cv2.imread('test.jpeg')


    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    result, encimg = cv2.imencode('.jpg', img, encode_param)
    buf=encimg.reshape((encimg.shape[0]))
    res_buf=insertMetaData(buf,[])


    print(len(buf),len(res_buf))






