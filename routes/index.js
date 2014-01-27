var _ = require('lodash');
var files = ['_index', 'interactive_marker_test', 'interactive_marker_test_json'];

exports.routes = _.map(files, function(f) {
    return require('./' + f);
});
