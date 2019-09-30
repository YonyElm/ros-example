pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                echo 'Buildings..'
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
        stage('Post_Build') {
            steps{
                sh  '''#!/bin/bash
                        curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=8f7137b10db2ca412225b54f9e8755b630f21cc7>" \
                            -H "Content-Type: application/json" \
                            -X POST \
                            -d "{\"state\": \"success\",\"context\": \"jenkins\", \"description\": \"Jenkins\", \"target_url\": \"http://52.31.241.174:8080/job/github_link/$BUILD_NUMBER/console\"}"
                    '''
                }
        }
    }
}