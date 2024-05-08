from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import numpy as np

import cv2

host = ('172.19.137.237', 8001)     # 机房环境下的主机ip
# host = ('172.31.95.73', 8002)  # 宿舍环境下的主机ip

class MyHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        datastr = self.rfile.read(int(self.headers['Content-Length'])).decode('utf-8').replace('[', '').replace(']', '')
        # TODO: 从规则引擎转发的json解析得到视频流地址，使用对应算法处理，
        js = json.loads(datastr)
        print(js["message"])

if __name__ == '__main__':
    server = HTTPServer(host, MyHandler)
    print('Server Started')
    server.serve_forever()