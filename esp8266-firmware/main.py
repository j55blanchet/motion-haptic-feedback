# Copied and modified from https://RandomNerdTutorials.com
import socket
import tape

def web_page():
  html = """<html><head> <meta charset="UTF-8"><title>Haptic Feedback Tape</title> <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
  h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none; 
  border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
  .button2{background-color: #4286f4;}
  .button:disabled{background-color: #444;}</style></head>
  <body> 
    <h1>ESP Web Server</h1> 
    <p><a href="/?command=extend"><button class="button">EXTEND</button></a></p>
    <p><a href="/?command=flex"><button class="button button2">FLEX</button></a></p>
    <script> 
        document.querySelectorAll('button').forEach(y => {
            y.onclick = () => {
                document.querySelectorAll('button').forEach((x) => {
                    x.setAttribute('disabled', true);
                });
            }
        });
    </script>
  </body></html>"""
  return html

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(5)

while True:
  conn, addr = s.accept()
  print('Got a connection from %s' % str(addr))
  request = conn.recv(1024)
  request = str(request)
  print('Content = %s' % request)
  extend_control = request.find('/?command=extend')
  flex_control = request.find('/?command=flex')
  if extend_control == 6:
    print('EXTEND CONTROL')
    tape.show_extension_sequence()
    
  if flex_control == 6:
    print('FLEX CONTROL')
    tape.show_retraction_sequence()

  response = web_page()
  conn.send('HTTP/1.1 200 OK\n')
  conn.send('Content-Type: text/html\n')
  conn.send('Connection: close\n\n')
  conn.sendall(response)
  conn.close()
