import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';
//import RobotWindow from 'botWindow.js';

// Initialize the RobotWindow class in order to communicate with the robot.
window.onload = function() {
  log('HTML page loaded.');
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('AGV UI');
  window.robotWindow.receive = receive;
};

// Log a message in the console widget.
window.log = function(message) {
  var ul = document.getElementById('console');
  var li = document.createElement('li');
  li.appendChild(document.createTextNode(message));
  ul.appendChild(li);
}

//Start/stop button
// need modifications
var startBit = 1;
window.toggleStopCheckbox =  function(obj) {
  if (obj.checked) {
    obj.parentNode.classList.add('checked');
    obj.parentNode.lastChild.innerHTML = 'Start Motors';
    //window.robotWindow.send('stop motors');
    startBit = 0;
    //log('Stop motors.');
  } else {
    obj.parentNode.classList.remove('checked');
    obj.parentNode.lastChild.innerHTML = 'Stop Motors';
    //window.robotWindow.send('release motors');
    startBit = 1;
    //log('Release motors.');
  }
  if (startBit == 0) {
    window.robotWindow.send('00000');
  }
  if (startBit == 1) {
    window.robotWindow.send('10000');
  }
  
}


