from pyramid.response import Response
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
    Vars['progress'] = 0
    return {'ok': True, 'project': 'mars'}

@view_config(route_name='rover_status', renderer='json')
def rover_status_view(request):
    if Vars['progress'] < 99:
        Vars['progress'] += 1
        return {'ok': True, 'status': 'stabdby', 'progress': Vars['progress'] }
    else:
        Vars['progress'] += 1
        return {'ok': True, 'status': 'completed', 'progress': Vars['progress'] }

@view_config(route_name='rasp', renderer='json')
def rasp_view(request):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('192.168.0.105', 10000)
    print('connecting to %s port %s' % server_address, file=sys.stderr)
    sock.connect(server_address)
    datiz = []
    try:
        message = 'This is the message.  It will be repeated. E quando arriva vabbenen'
        print('sending "%s"' % message, file=sys.stderr)
        sock.sendall(message.encode())
        amount_received = 0
        amount_expected = len(message)
        while amount_received < amount_expected:
            data = sock.recv(16)
            amount_received += len(data)
            datiz.append(str(data))
            print('received "%s"' % data, file=sys.stderr)

    finally:
        print('closing socket', file=sys.stderr)
        sock.close()

    return {'ok': True, 'data': "ricevuti"}


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

