// routes/interactive_marker_test_json.js

var mongoose = require('mongoose');
var InteractiveTestRecord = mongoose.model('InteractiveTestRecord');

exports.get_url = '/interactive_marker_test/json';

exports.get = function(req, res, next) {
  InteractiveTestRecord
    .find()
    .sort([['time', 'ascending']])
    .exec(function(err, records) {
      res.send(records);
    });
};
