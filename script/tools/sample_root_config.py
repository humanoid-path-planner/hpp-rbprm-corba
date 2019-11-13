import random
random.seed()

def generate_random_conf_without_orientation(rbprmBuilder,bounds,v=None):
    q = rbprmBuilder.getCurrentConfig ();
    while True:
      pos = [0,0,0]
      for i in range(3):
        pos[i] = random.uniform(bounds[i*2],bounds[i*2+1])
      quat = [0,0,0.7071, 0.7071]
      q[0:3] = pos
      q[3:7] = quat
      q[8] = 0.006761
      q[-6:-3] = [0,0,0]
      if v :
        v(q)
      status,message = rbprmBuilder.isConfigValid(q)
      if status:
        quat = [0,0, 0.382911, 0.923785]
        q[3:7] = quat
        status,message = rbprmBuilder.isConfigValid(q)
      if status:
        quat = [0,0, -0.382911, 0.923785]
        q[3:7] = quat
        status,message = rbprmBuilder.isConfigValid(q)
      if status:
        quat = [0,0,0, 1.]
        q[3:7] = quat
        status,message = rbprmBuilder.isConfigValid(q)
      if status:
        return q
      else:
        print "Getting invalid config. try again."
        print message


def generate_random_conf_with_orientation(rbprmBuilder,bounds,v_init = 0.1,v=None):
    q = rbprmBuilder.getCurrentConfig ();
    while True:
      pos = [0,0,0]
      for i in range(3):
        pos[i] = random.uniform(bounds[i*2],bounds[i*2+1])
      angle = random.uniform(0.,2.*np.pi)
      quat = np.array([0,0,np.sin(angle/2), np.cos(angle/2)])
      q[0:3] = pos
      q[3:7] = quat
      q[8] = 0.006761
      q[-6:-3] = [v_init*np.cos(angle),v_init*np.sin(angle),0]
      if v :
        v(q)
      status,message = rbprmBuilder.isConfigValid(q)
      if status:
        return q
      else:
        print "Getting invalid config. try again."
        print message
