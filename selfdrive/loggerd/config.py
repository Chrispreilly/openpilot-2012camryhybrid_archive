import os

if os.environ.get('LOGGERD_ROOT', False):
  ROOT = os.environ['LOGGERD_ROOT']
  print("Custom loggerd root: ", ROOT)
else:
  ROOT = '/data/media/0/realdatatemp/'

SEGMENT_LENGTH = 60
