#! /usr/bin/env python
from flask import Flask, render_template, request, url_for, flash, make_response, redirect
import Queue
import threading
import time
# import rospy
# import turtlebot_teleop.msg
# from std_msgs.msg import String

pub = None

# Reuqests are stored in the dictionary in the following way:
# Requester name -> (item, location, number in queue, completed)
# Each request takes ~1 minute so the number in queue becomes the est wait time
request_statuses = dict()

# Dictionary of usernames
usernames = dict()

queued_requests = 0

# Initialize the Flask application
app = Flask(__name__)

# Define a route for the default URL, which loads the form
@app.route('/')
def form():
    userCookie = request.cookies.get('userID')
    if userCookie in usernames:
        return render_template('request_form_full.html', name=userCookie)
    return render_template('index.html')

# ------ Debugging Method ---------
def simulate_complete(name):
    print "Timer elapsed"
    actionServer = JobCompletionNotification(name)
    actionServer.execute_cb(None)

@app.route('/loginUser/', methods=['POST'])
def loginUser():
    print "Trying to log user in"
    username = request.form['username']
    password = request.form['password']
    print "User is %s:%s", (username, password)
    if username in usernames:
        if usernames[username] == password:
            # Set our cookie and return the request
            # resp = make_response(render_template('request_form_full.html', name=username))
            resp = make_response(render_template('request_form_minimal.html', name=username))
            resp.set_cookie('userID', username)
            return resp
            
    return "failure"
            
            
# Probably will not need to use this method at all
# @app.route('/user/<name>')
# def user(name):
#     username = request.cookies.get('userID')
#     if name != username:
#         # failure
#         flash("You are not authorized")
#         return redirect(url_for('/'))
#     else:
#         return "Logged in!"
#         pass

@app.route('/itemRequest/', methods=['POST'])
def itemRequest():
    item = request.form['item']
    location = request.form['location']
    name = request.cookies.get('userID')
    
    # Now that we've informed the user, publish the item
    # to a topic so our robot can run it
    
    if name not in request_statuses:
        # add one to the queue count
        global queued_requests
        queued_requests += 1
        request_statuses[name] = (item, location, queued_requests, False)
        
        # global pub
        # pub.publish(str.format('{0}\t{1}\t{2}', name, item, location))
        
        # -------- Below is debug code for testing purposes only -----------
        print "Starting 10 second timer for request: %s" % name 
        timer = threading.Timer(10, simulate_complete, [name])
        timer.start()
    else:
        print "Name already in request statuses"
        flash('You already have a request submitted. Please complete it before submitting a new one')
        print request_statuses
    
    return render_template('request_submit.html', item=item, location=location, name=name)
    
@app.route('/show_status/<name>', methods=['GET'])
def show_status(name):
    if name not in request_statuses:
        return "There is no request for %s<br />It was either already completed or timed out" % name
    _, _, request_num, completed = request_statuses[name]
    if completed:
        print "Status success"
        return "Your item is ready at its destination!"
    else:
        print "Status failure"
        return "You are currently number %s in the queue.<br />Estimated time remaining: %s minute(s)" % (str(request_num), str(request_num*2))

@app.route('/end_request/<name>', methods=['POST'])
def end_request(name):
    global request_statuses
    if name not in request_statuses:
        flash('Your request could not be found!')
        flash('It has either already been completed or has timed out.')
        return render_template('form_submit.html')
    
    item, location, queue_num, completed = request_statuses[name]
    if not completed:
        flash('Your request has not completed yet!')
        return render_template('form_submit.html', name=name, item=item, location=location)
    
    del request_statuses[name]
    
    global queued_requests
    queued_requests -= 1
    
    for key in request_statuses:
        req_item, req_loc, req_num, req_complete = request_statuses[key]
        request_statuses[key] = (req_item, req_loc, req_num-1, req_complete)
        
    flash("Thank you for picking up your item!")
    return render_template('form_submit.html')
    
    
class JobCompletionNotification(object):
    # create messages that are used to publish feedback/result
    # _feedback = turtlebot_teleop.msg.UserCompleteFeedback()
    # _result   = turtlebot_teleop.msg.UserCompleteResult()

    def __init__(self, name):
        self._action_name = name
        # self._as = actionlib.SimpleActionServer(self._action_name, turtlebot_teleop.msg.UserCompleteAction, execute_cb=self.execute_cb, auto_start = False)
        # self._as.start()
        
    def execute_cb(self, goal):
        global request_statuses
        # helper variables
        # r = rospy.Rate(1) # helps sleep at a set rate
        success = False
        
        # set initial cycle count
        # self._feedback.count = 0
        
        # publish info to the console for the user
        # rospy.loginfo('%s: Executing, ending of a user request %s' % (self._action_name, goal.job))
        
        # Set completed to true
        # item, location, queue_num, completed = request_statuses[goal.job]
        # request_statuses[goal.job] = (item, location, queue_num, True)
        
        # --------------- DEBUG CODE -------------------
        item, location, queue_num, completed = request_statuses[self._action_name]
        request_statuses[self._action_name] = (item, location, queue_num, True)
        
        
        
        for i in range(20):
            # Job deleted so we can break
            # User clicked done
            # if goal.job not in request_statuses:
            if self._action_name not in request_statuses:
                success = True
                break
            
            # Completed cycle unsucessfully
            # self._feedback.count += 1
            # self._as.publish_feedback(self._feedback)
            # r.sleep() # Sleep 1 second before checking again
            
            # ------------ Debug code -------------
            time.sleep(1)
            
        
        
        # self._result.suceeded = 1 if success else 0
        if success:
        #    rospy.loginfo('%s: Succeeded' % self._action_name)
            print "User Clicked action completion"
        else:
            print "User did not complete action"
            del request_statuses[self._action_name]
    
            global queued_requests
            queued_requests -= 1
            
            for key in request_statuses:
                req_item, req_loc, req_num, req_complete = request_statuses[key]
                request_statuses[key] = (req_item, req_loc, req_num-1, req_complete)
        # self._as.set_succeeded(self._result)

# Run the app
if __name__ == '__main__':    
    # Publish requests to the channel in the string representation of:
    # Requester name      item      location
    # where the delimeter is a tab. The requester name becomes the key
    # for the request, as each requester can only have one outstanding request
    # pub = rospy.Publisher('item_requests', String, queue_size=0)
    
    # rospy.init_node('web_app')
    usernames['test'] = 'test'
    app.secret_key = 'super secret key'
    app.run( 
            host="0.0.0.0",
            port=int("33333")
    )
