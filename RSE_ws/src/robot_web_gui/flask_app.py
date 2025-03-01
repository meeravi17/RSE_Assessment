from flask import Flask, render_template, url_for,g,jsonify,request
from sqlite3 import Error
import subprocess
import signal
import os
import time
import sqlite3


app = Flask(__name__)



DATABASE = os.path.join(os.getcwd(), "static", "database.db")
def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
    return db

		
class roslaunch_process():
    @classmethod
    def start_navigation(cls, mapname):
        subprocess.run(["tmux", "kill-session", "-t", "mapping_bash"])
        subprocess.run(["tmux", "kill-session", "-t", "navigation_bash"])
    
        bash_script_path = "~/RSE_Assessment/RSE_ws/src/scripts/standalone_navigation/localization_nd_navigation.bash"
        expanded_path = os.path.expanduser(bash_script_path)
    
        # Define your arguments
        arg1 = "map_file:=" + os.getcwd() + "/static/" + mapname + ".yaml"
        cls.process_navigation = subprocess.Popen(["bash", expanded_path, arg1])

    @classmethod
    def stop_navigation(self):
        self.process_navigation = subprocess.run(["tmux", "kill-session", "-t", "navigation_bash"])

    @classmethod
    def start_mapping(cls):
        cls.process_mapping = subprocess.run(["tmux", "kill-session", "-t", "mapping_bash"])
        bash_script_path = "~/RSE_Assessment/RSE_ws/src/scripts/standalone_navigation/mapping_scripts.bash"
        expanded_path = os.path.expanduser(bash_script_path)
        
        cls.process_mapping = subprocess.Popen(["bash", expanded_path])

    @classmethod
    def stop_mapping(self):
        self.process_mapping = subprocess.run(["tmux", "kill-session", "-t", "mapping_bash"])
    
@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

@app.before_request
def create_table():
    with app.app_context():
        try:
            db = get_db()
            c = db.cursor()
            c.execute("CREATE TABLE IF NOT EXISTS maps (id INTEGER PRIMARY KEY, name TEXT NOT NULL)")
            db.commit()
            c.close()
        except Error as e:
            print(e)


@app.route('/')
def index():
	
	with get_db():
	    try:
	        c = get_db().cursor()
	        c.execute("SELECT * FROM maps")
        	data = c.fetchall()
	        c.close()
	    except Error as e:
	        print(e)
	

	return render_template('index.html',title='Index',map = data)


@app.route('/index/<variable>',methods=['GET','POST'])
def themainroute(variable):
	if variable == "navigation-precheck" :

		with get_db():


	        	try:
	        
		            c = get_db().cursor()
		           
		            c.execute("SELECT count(*) FROM maps")
		            k=c.fetchall()[0][0]
		            c.close()
		           
		            print(k)
		            return jsonify(mapcount=k) 
	                
	            
	        	except Error as e:
	            		print(e)
	elif variable == "gotonavigation":

		mapname =request.get_data().decode('utf-8')

		roslaunch_process.start_navigation(mapname)
		
		return "success"

		    
@app.route('/navigation',methods=['GET','POST'])

def navigation():

	with get_db():
	    try:
	        c = get_db().cursor()
	        c.execute("SELECT * FROM maps")
        	data = c.fetchall()
	        c.close()
	    except Error as e:
	        print(e)
	return render_template('navigation.html',map = data)



@app.route('/navigation/deletemap',methods=['POST'])
def deletemap():
	mapname = request.get_data().decode('utf-8')
	print(mapname)
	os.system("rm -rf"+" "+os.getcwd()+"/static/"+mapname+".yaml "+os.getcwd()+"/static/"+mapname+".png "+os.getcwd()+"/static/"+mapname+".pgm")

	with get_db():
	    try:
	        c = get_db().cursor()
	        c.execute("DELETE FROM maps WHERE name=?", (mapname,))
	        c.close()
	    except Error as e:
	        print(e)
	return ("successfully deleted map")	


@app.route("/navigation/<variable>" , methods=['GET','POST'])
def gotomapping(variable):
	# return "success"
	if variable == "index":
		roslaunch_process.start_mapping()
	elif variable == "gotomapping":		
		roslaunch_process.stop_navigation()
		time.sleep(2)
		roslaunch_process.start_mapping()
	return "success"



@app.route("/navigation/loadmap" , methods=['POST'])
def navigation_properties():

	mapname = request.get_data().decode('utf-8')
	
	roslaunch_process.stop_navigation()
	time.sleep(5)
	roslaunch_process.start_navigation(mapname)
	return("success")


@app.route("/navigation/stop" , methods=['POST'])
def stop():
	os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}") 
	return("stopped the robot")


@app.route('/mapping')
def mapping():
	with get_db():
	    try:
	        c = get_db().cursor()
	        c.execute("SELECT * FROM maps")
        	data = c.fetchall()
	        c.close()
	    except Error as e:
	        print(e)
	
	return render_template('mapping.html', title='Mapping', map = data) 
	


@app.route("/mapping/cutmapping" , methods=['POST'])
def killnode():
	roslaunch_process.stop_mapping() 
	return("killed the mapping node")



@app.route("/mapping/savemap" , methods=['POST'])
def savemap():
	mapname = request.get_data().decode('utf-8')

	os.system("rosrun map_server map_saver -f"+" "+os.path.join(os.getcwd(),"static",mapname))
	os.system("convert"+" "+os.getcwd()+"/static/"+mapname+".pgm"+" "+os.getcwd()+"/static/"+mapname+".png")
	
	with get_db():
	    try:
	        c = get_db().cursor()
	        c.execute("insert into maps (name) values (?)", (mapname,))
	        # get_db().commit()
	        c.close()
	    except Error as e:
	        print(e)

	return("success")


@app.route("/shutdown" , methods=['POST'])
def shutdown():
	os.system("shutdown now") 
	return("shutting down the robot")	




@app.route("/restart" , methods=['POST'])
def restart():
	os.system("restart now") 
	return("restarting the robot")





if __name__ == '__main__':
	app.run(debug=True)    
