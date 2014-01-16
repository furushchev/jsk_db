// models/interactive_test_record.js

var mongoose = require('mongoose');
var Schema = mongoose.Schema;

var InteractiveTestRecordSchema = new Schema({
  created_at: Date,
  user: String,
  ui_type: String,
  time: Number
});

mongoose.model('InteractiveTestRecord', InteractiveTestRecordSchema);

var InteractiveTestRecord = mongoose.model('InteractiveTestRecord');

InteractiveTestRecordSchema.pre('save', function(next) {
  var record = this;
  var now = new Date();
  record.created_at = now;
  next();
});
