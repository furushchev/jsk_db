// routes/interactive_marker_test

var mongoose = require('mongoose');
var InteractiveTestRecord = mongoose.model('InteractiveTestRecord');

exports.get_url = '/interactive_marker_test';
exports.get = function(req, res, next) {
  InteractiveTestRecord
    .find()
    .sort([['time', 'ascending']])
    .exec(function(err, records) {
    if (err != null) {
      next(err);
    }
    else {
      // sort the records
      res.render('interactive_marker_test', {records: records});
    }
  });
};

exports.post_url = '/interactive_marker_test';
exports.post = function(req, res, next) {
  // POST parameters
  // user: string
  // ui_type: string
  // time: string
  var user = req.param('user');
  var ui_type = req.param('ui_type');
  var time = parseFloat(req.param('time'));
  console.log('user: ' + user);
  console.log('ui_type: ' + ui_type);
  console.log('time: ' + time);
  
  var record = new InteractiveTestRecord({
    user: user,
    ui_type: ui_type,
    time: time
  });
  record.save(function(err) {
    if (err == null) {
      console.log('success to save the record');
    }
  });
  res.send(200);
};
