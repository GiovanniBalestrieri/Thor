from pyramid.config import Configurator
from sqlalchemy import engine_from_config

from .models import (
    DBSession,
    Base,
    )


def main(global_config, **settings):
    """ This function returns a Pyramid WSGI application.
    """
    engine = engine_from_config(settings, 'sqlalchemy.')
    DBSession.configure(bind=engine)
    Base.metadata.bind = engine
    config = Configurator(settings=settings)
    config.include('pyramid_chameleon')
    config.add_static_view('static', 'static', cache_max_age=3600)
    config.add_route('home', '/')
    config.add_route('rover_start', '/rover/start')
    config.add_route('rover_status', '/rover/status')
    config.add_route('rover_photo', '/rover/photo')
    config.add_route('rover_obstacles', '/rover/obstacles')
    config.add_route('rover_path', '/rover/path')
    config.add_route('rover_go', '/rover/go')
    config.add_route('rover_position', '/rover/position')
    config.scan()
    return config.make_wsgi_app()
