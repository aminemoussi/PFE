python -c "import carla,time; c=carla.Client('localhost',2000); c.set_timeout(600); print('loading...'); c.load_world('Town03'); time.sleep(10); print('Town03 ready')"

cd PythonAPI/util && python config.py --no-rendering
