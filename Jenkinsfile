pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                echo 'Building..'
            }
        }
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        stage('Deploy') {
            steps {
                echo 'Deploying....'
            }
        }
    }

    // Post build - will be executed after build is done
    post {
        success{
            sh  '''#!/bin/bash
                    echo $GIT_COMMIT
                    echo $BUILD_NUMBER
                    curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=4e2984b191e61afa65b6e4b044cf321846e164f8" \
                        -H "Content-Type: application/json" \
                        -X POST \
                        -d '{"state": "success", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                '''
        }
        failure{
            sh  '''#!/bin/bash
                    echo $GIT_COMMIT
                    echo $BUILD_NUMBER
                    curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=4e2984b191e61afa65b6e4b044cf321846e164f8" \
                        -H "Content-Type: application/json" \
                        -X POST \
                        -d '{"state": "failure", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                '''
        }
    }
}
