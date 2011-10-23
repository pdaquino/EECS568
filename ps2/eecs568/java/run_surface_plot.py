def do_config(odom, lm, base_config):
    s = "noisemodels {\n"
    s = s+" odometryDiag = [" + ("%.15f" % odom) + ","+ ("%.15f" % odom) + "];\n"
    s = s+" landmarkDiag = [" + ("%.15f" % lm) + ","+ ("%.15f" % (0.1*lm)) + "];\n}\n"
    s = s+base_config
    return s


def make_configs(odoms, lms):
    config = open('../config/plot_sim.config', 'r').read()
    i = 0
    for odom in odoms:
        for lm in lms:
            new_config = do_config(odom, lm, config)
            open('temp/temp_config_'+str(i)+'.config','w').write(new_config)
            i = i+1
            
def do_bash(n):
    s_odd = ""
    s_even = ""
    for i in range(0,n):
        if i % 2 == 0:
            s_even = s_even+"java -classpath april.jar april.sim.Simulator -c temp/temp_config_"+str(i)+".config -l april.sim.LeastSquaresNoAssocListener > output/out_"+str(i)+".txt\n"
        else:
            s_odd = s_odd+"java -classpath april.jar april.sim.Simulator -c temp/temp_config_"+str(i)+".config -l april.sim.LeastSquaresNoAssocListener > output/out_"+str(i)+".txt\n"
    open("run_surface_plot_sims_odd.sh", "w").write(s_odd)
    open("run_surface_plot_sims_even.sh", "w").write(s_even)
            
def get_values(base):
    return [0.75*0.75*base, 0.75*base, base, 1.25*base, 1.25*1.25*base]

def run():
    odoms = get_values(0.0424)
    lms = get_values(0.001)
    make_configs(odoms, lms)
    do_bash(len(odoms)*len(lms))
    
run()
            
			
