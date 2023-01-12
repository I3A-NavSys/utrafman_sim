module.exports = {
    apps : [{
      name   : "SIAM_HTTP_Manager",
      script : "./app.js",
      watch: true,
      log_date_format: "DD-MM-YYYY HH:mm:ss Z",
      env_production: {
        NODE_ENV: "production"
     }
    }]
  }