//
//  ViewController.swift
//  ExerciseBand
//
//  Created by Kireet Agrawal on 4/15/17.
//  Copyright © 2017 Kireet Agrawal. All rights reserved.
//

import UIKit
import CoreMotion
import AVFoundation

class ViewController: UIViewController {
    
    //Core Motion Instance Variables
    var currentAccelX: Double = 0.0
    var currentAccelY: Double = 0.0
    var currentAccelZ: Double = 0.0
    
    var currentRotX: Double = 0.0
    var currentRotY: Double = 0.0
    var currentRotZ: Double = 0.0
    
    var movementManager = CMMotionManager()
    let speechSynth = AVSpeechSynthesizer()
    var player: AVAudioPlayer?
    
    var counter: Int = 0
    var down: Bool = true
    

    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        currentAccelX = 0
        currentAccelY = 0
        currentAccelZ = 0
        
        currentRotX = 0
        currentRotY = 0
        currentRotZ = 0
        
        movementManager.gyroUpdateInterval = 0.1
        movementManager.accelerometerUpdateInterval = 0.1
        
        //Start Recording Data
        movementManager.startAccelerometerUpdates(to: OperationQueue.current!) { (accelerometerData: CMAccelerometerData?, NSError) -> Void in
            
            self.outputAccData(acceleration: accelerometerData!.acceleration)
            if(NSError != nil) {
                //print("\(NSError)")
            }
        }
        
        movementManager.startGyroUpdates(to: OperationQueue.current!, withHandler: { (gyroData: CMGyroData?, NSError) -> Void in
            self.outputRotData(rotation: gyroData!.rotationRate)
            if (NSError != nil){
                //print("\(NSError)")
            }
            
        })

    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func outputAccData(acceleration: CMAcceleration){
        
        currentAccelX = acceleration.x
        currentAccelY = acceleration.y
        currentAccelZ = acceleration.z
        
        
        if (fabs(pow(acceleration.z, 2.0)) + fabs(pow(acceleration.y, 2.0)) + fabs(pow(
            acceleration.x, 2.0)) > 4.5) {
            
            //counter += 1
            speak(textToSpeak: "Jump")
        }
        
        if (fabs(acceleration.z) > 0.7) {
            counter += 1
        }
        
    }
    
    func outputRotData(rotation: CMRotationRate){
        
        currentRotX = rotation.x
        currentRotY = rotation.y
        currentRotZ = rotation.z
        
        if (fabs(pow(rotation.z, 2.0)) + fabs(pow(rotation.y, 2.0)) + fabs(pow(
            rotation.x, 2.0)) > 3) {
            
            print(rotation.z)
            print(rotation.x)
            print(rotation.y)
            
            //counter += 1
            //speak(textToSpeak: "Keep going")
        }
    }
    
    func playSound(songName: String) {
        guard let sound = NSDataAsset(name: songName) else {
            print("asset not found")
            return
        }
        
        do {
            try AVAudioSession.sharedInstance().setCategory(AVAudioSessionCategoryPlayback)
            try AVAudioSession.sharedInstance().setActive(true)
            
            player = try AVAudioPlayer(data: sound.data, fileTypeHint: AVFileTypeMPEGLayer3)
            
            player!.play()
        } catch let error as NSError {
            //print("error: \(error.localizedDescription)")
        }
    }
    
    func speak(textToSpeak: String) {
        let speech = AVSpeechUtterance(string: textToSpeak)
        speech.pitchMultiplier = 0.9
        speechSynth.speak(speech)
    }
    
    func stopSound() {
        player!.stop()
    }


}

