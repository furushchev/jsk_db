
/**
 * Module dependencies.
 */

var express = require('express');
var user = require('./routes/user');
var http = require('http');
var path = require('path');
var mongoose = require('mongoose');
var app = express();

if (process.env.MONGOLAB_URI) {
  mongoose.connect(process.env.MONGOLAB_URI);
}
else {
  mongoose.connect('mongodb://localhost/jsk_db');
}

require('./models');

// all environments
app.set('port', process.env.PORT || 3000);
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');
app.use(express.favicon());
app.use(express.logger('dev'));
app.use(express.json());
app.use(express.urlencoded());
app.use(express.methodOverride());
app.use(app.router);
app.use(express.static(path.join(__dirname, 'public')));

// development only
if ('development' == app.get('env')) {
  app.use(express.errorHandler());
}

// app.get('/', routes.index);
// app.get('/users', user.list);

// routing
var routes = require("./routes");
routes.routes.forEach(function(r) {
  if (r.get_url && r.get) {
    if (r.get_url instanceof Array) {
      r.get_url.forEach(function(u) {
        app.get(u, r.get);
      });
    }
    else {
      app.get(r.get_url, r.get);
    }
  }
  if (r.post_url && r.post) {
    if (r.post_url instanceof Array) {
      r.post_url.forEach(function(u) {
        app.post(u, r.post);
      });
    }
    else {
      app.post(r.post_url, r.post);
    }
  }
});


http.createServer(app).listen(app.get('port'), function(){
  console.log('Express server listening on port ' + app.get('port'));
});
