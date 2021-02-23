from __future__ import print_function
import subprocess, resource
import os
import time
from shutil import copyfile
from timeit import timeit
import signal
import psutil
from datetime import datetime
import pandas


CONF_FOLDER = "/opt/workspace/simulation/mars/plugins/envire_mls/testMlsData/"
tests = [
  "debug.yml",
  "001_entrance_104cm.yml",
  #"002_entrance_244cm_1_5radS.yml",
  ##"002_entrance_248cm.yml", # NOK for 1.5 rad/s
  #"003_low_ceiling_01_127cm.yml",
  #"004_low_ceiling_02_242cm.yml",
  #"005_low_ceiling_03_219cm.yml",
  #"006_low_ceiling_04_219cm.yml",
  #"007_before_first_slope_01_222cm.yml",
  #"008_start_first_slope_01_403cm.yml", # OK for 1.5 rad/s 
  ##"008_start_first_slope_01_391cm.yml", # NOK for 1.5 rad/s
  #"009_on_first_slope_333cm.yml" 
  ] 
distances = {
  "debug.yml" : 0.0,
  "001_entrance_104cm.yml": 104,
  "002_entrance_248cm.yml": 248,
  "002_entrance_244cm_1_5radS.yml": 244,
  "003_low_ceiling_01_127cm.yml": 127,
  "004_low_ceiling_02_242cm.yml": 242,
  "005_low_ceiling_03_219cm.yml": 219,
  "006_low_ceiling_04_219cm.yml": 219,
  "007_before_first_slope_01_222cm.yml": 222,
  "008_start_first_slope_01_403cm.yml": 403,
  "008_start_first_slope_01_391cm.yml": 391,
  "009_on_first_slope_333cm.yml": 333
}
speed = 0.2



def runTestNoTime(path_to_copy):
    if os.path.exists(CONF_FOLDER + "conf.yml"):
      os.remove(CONF_FOLDER + "conf.yml")
    copyfile(path_to_copy, CONF_FOLDER+"conf.yml")
    usage_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    start_time = datetime.now()
    cpu_time = -1
    duration = -1
    #p1 = subprocess.Popen(['mars_app'], preexec_fn=os.setsid, shell=True, stderr=subprocess.PIPE, universal_newlines=True)
    #p1 = subprocess.Popen(['mars_app'], shell=True, stderr=subprocess.PIPE, universal_newlines=True)
    p1 = subprocess.Popen(['mars_app'], stderr=subprocess.PIPE, universal_newlines=True)
    parent = psutil.Process(p1.pid)
    #for stdout_line in iter(p1.stdout.readline, ""):
    #  #print(stdout_line)
    #  print("LALALA")
    for stderr_line in iter(p1.stderr.readline, ""):
      if "[EnvireMls::update] Goal was reached" in stderr_line:
        print(stderr_line)
        #p1.stdout.close()
        duration = (datetime.now() - start_time).total_seconds()
        usage_end = resource.getrusage(resource.RUSAGE_CHILDREN)
        cpu_time = usage_end.ru_utime - usage_start.ru_utime
        for child in parent.children(recursive=True):
          child.terminate()
        parent.terminate()
        #os.killpg(os.getpgid(p1.pid), signal.SIGTERM)
        print("Process killed")
    return (cpu_time, duration)



def runTest(path_to_copy):
    if os.path.exists(CONF_FOLDER + "conf.yml"):
      os.remove(CONF_FOLDER + "conf.yml")
    copyfile(path_to_copy, CONF_FOLDER+"conf.yml")
    usage_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    duration = timeit(stmt = "subprocess.call('mars_app')", setup = "import subprocess", number = 1)
    usage_end = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_time = usage_end.ru_utime - usage_start.ru_utime
    return (cpu_time, duration)


res = pandas.DataFrame(index=tests, columns=["cpu_time", "duration", "speed", "distance"])

for test in tests:
    #(cpu_time, real_time) = runTest(CONF_FOLDER+test)
    (cpu_time, duration) = runTestNoTime(CONF_FOLDER+test)
    #for mess in runTestNoTime(CONF_FOLDER+test):
    #  if "[EnvireMls::update] Goal was reached" in mess:
    #    print("**************************** GOAL REACHED! ****************************")
    res.at[test, "cpu_time"] = cpu_time
    res.at[test, "distance"] = distances[test]
    res.at[test, "duration"] = duration
    res.at[test, "speed"] = speed
    #raw_input("Press key to proceed with the next test")

print(res.to_latex(index=False))
print(res)

# Using time I get these values for the debug scenario
#real	0m26.905s
#user	0m21.426s
#sys	0m1.236s



#{'conf_2.yml': (197.17293199999997, 134.77659487724304), 'conf_1.yml': (161.670422, 103.97521901130676)}
#

#p1 = subprocess.Popen(['time', 'mars_app'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#(stdout, stderr) = p1.communicate()
#print("CPU Time: " + str(cpu_time))
#
#
#os.remove(CONF_FOLDER + "conf.yml")
#copyfile(CONF_FOLDER + "conf_2.yml", CONF_FOLDER+"conf.yml")
#time_output = StringIO.StringIO()
#subprocess.call(["time", "mars_app"])#, stdout=time_output)
#print(time_output)
#print timeit(stmt = "subprocess.call('mars_app')", setup = "import subprocess", number = 1)