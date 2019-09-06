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
        stage('Post_Build') {
            steps{
                bash '''#!/bin/bash
                        curl "www.google.com"
                    '''
                }
        }
    }
}