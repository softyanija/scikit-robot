import os
import os.path as osp

import gdown


data_dir = osp.abspath(osp.dirname(__file__))
_default_cache_dir = osp.expanduser('~/.skrobot')


def get_cache_dir():
    return os.environ.get('SKROBOT_CACHE_DIR', _default_cache_dir)


def bunny_objpath():
    target_path = osp.join(get_cache_dir(), 'mesh', 'bunny.obj')
    gdown.cached_download(
        url='https://raw.githubusercontent.com/iory/scikit-robot-models/master/data/bunny.obj',  # NOQA
        path=target_path,
        md5='19bd31bde1fcf5242a8a82ed4ac03c72',
        quiet=True,
    )
    return target_path


def fetch_urdfpath():
    gdown.cached_download(
        url='https://github.com/iory/scikit-robot-models/raw/master/fetch_description.tar.gz',  # NOQA
        path=osp.join(get_cache_dir(), 'fetch_description.tar.gz'),
        md5='fbe29ab5f3d029d165a625175b43a265',
        postprocess=gdown.extractall,
        quiet=True,
    )
    return osp.join(get_cache_dir(), 'fetch_description', 'fetch.urdf')


def kuka_urdfpath():
    return osp.join(data_dir, 'kuka_description', 'kuka.urdf')


def panda_urdfpath():
    gdown.cached_download(
        url='https://github.com/iory/scikit-robot-models/raw/master/franka_description.tar.gz',  # NOQA
        path=osp.join(get_cache_dir(), 'franka_description.tar.gz'),
        md5='3de5bd15262b519e3beb88f1422032ac',
        postprocess=gdown.extractall,
        quiet=True,
    )
    return osp.join(get_cache_dir(), 'franka_description', 'panda.urdf')


def pr2_urdfpath():
    gdown.cached_download(
        url='https://github.com/softyanija/scikit-robot-models/raw/master/pr2_description.tar.gz',  # NOQA
        path=osp.join(get_cache_dir(), 'pr2_description.tar.gz'),
        md5='e4fb915accdb3568a5524c92e9c35c9a',
        postprocess=gdown.extractall,
        quiet=True,
    )
    return osp.join(get_cache_dir(), 'pr2_description', 'pr2.urdf')

def pr2hand_urdfpath():
    #WIP
    """
    gdown.cached_download(
        url='https://github.com/softyanija/scikit-robot-models/raw/master/pr2_description.tar.gz',  # NOQA
        path=osp.join(get_cache_dir(), 'pr2_description.tar.gz'),
        md5='e4fb915accdb3568a5524c92e9c35c9a',
        postprocess=gdown.extractall,
        quiet=True,
    )
    """
    return osp.join('/home/amabe/robothand_dataset/scikit-robot-models', 'pr2hand', 'pr2hand.urdf')
