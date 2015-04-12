from pyramid.response import Response, FileResponse
from pyramid.view import view_config

from sqlalchemy.exc import DBAPIError

from .models import (
    DBSession,
    MyModel,
    )

import socket
import sys


@view_config(route_name='home', renderer='templates/mytemplate.pt')
def my_view(request):
    try:
        one = DBSession.query(MyModel).filter(MyModel.name == 'one').first()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'one': one, 'project': 'mars'}

Vars = {}
Vars['progress'] = 0

@view_config(route_name='rover_start', renderer='json')
def rover_start_view(request):
    print('rover_start')
    Vars['progress'] = 0
    return {'ok': 1, 'project': 'mars'}

@view_config(route_name='rover_status', renderer='json')
def rover_status_view(request):
    print('rover_status')
    if Vars['progress'] < 99:
        Vars['progress'] += 1
        return {'ok': 1, 'status': 'stabdby', 'progress': Vars['progress'] }
    else:
        Vars['progress'] += 1
        return {'ok': 1, 'status': 'completed', 'progress': Vars['progress'] }

@view_config(route_name='rover_position', renderer='json')
def rover_position_view(request):
    print('rover_position')
    Vars['progress'] = 0
    return {'ok': 1, 'x': 12.156556, 'y': 19.566556}

@view_config(route_name='rover_photo', renderer='json')
def rover_photo_view(request):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('192.168.0.105', 10000)
    print('connecting to %s port %s' % server_address, file=sys.stderr)
    sock.connect(server_address)
    datiz = []
    pacchetto = 2048
    try:
        message = 'foto'
        print('sending "%s"' % message, file=sys.stderr)
        sock.sendall(message.encode())
        amount_received = pacchetto
        amount_expected = len(message)
        while amount_received == pacchetto:
            data = sock.recv(pacchetto)
            amount_received = len(data)
            datiz.append(data)
            print('received %d' % len(data), file=sys.stderr)

    finally:
        print('closing socket', file=sys.stderr)
        sock.close()
    f = open('fotina.jpg','wb')
    datix = b''.join(datiz)
    f.write(datix)
    f.close()

    # here = os.path.dirname(__file__)
    # filepath = os.path.join(here, 'swf', filename+".swf")
    return FileResponse('fotina.jpg', request=request, content_type='image/jpeg')


@view_config(route_name='rover_obstacles', renderer='json')
def rover_obstacles_view(request):
    return FileResponse('obstacles.png', request=request, content_type='image/png')


@view_config(route_name='rover_path', renderer='json')
def rover_path_view(request):
    print('rover_path')
    return {'ok': 1, 'data': 'all', 'more': 'data'}


@view_config(route_name='rover_go', renderer='json')
def rover_path_view(request):
    print('rover_go')
    return {'ok': 1, 'data': 'all', 'more': 'data'}


conn_err_msg = """\
Pyramid is having a problem using your SQL database.  The problem
might be caused by one of the following things:

1.  You may need to run the "initialize_mars_db" script
    to initialize your database tables.  Check your virtual
    environment's "bin" directory for this script and try to run it.

2.  Your database server may not be running.  Check that the
    database server referred to by the "sqlalchemy.url" setting in
    your "development.ini" file is running.

After you fix the problem, please restart the Pyramid application to
try it again.
"""

