import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import servocontrol
 
 
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print 'new connection'
        self.write_message("Hello World")
        self.servos = servocontrol.ServoController()

    def on_message(self, message):
        print 'message received %s' % message
        channel = int(message[0])
        if channel < 9:
           c = int(channel)
           value = int(message[1:])
           self.servos.setAngle(c, value)
           print "channel: %d angle: %d" % (c, value)
        else:
           self.servos.triggerScript(0)

    def on_close(self):
      print 'connection closed'
 
 
application = tornado.web.Application([
    (r'/ws', WSHandler),
])
 
 
if __name__ == "__main__":
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()