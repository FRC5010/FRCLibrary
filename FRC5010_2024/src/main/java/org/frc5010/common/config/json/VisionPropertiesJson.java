// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class VisionPropertiesJson {
	public String[] cameras;
	
	public void createCameraSystem(GenericRobot robot, Map<String, CameraConfigurationJson> map) {
		map.keySet().forEach(it -> {
			map.get(it).configureCamera(robot);
		});
	}

	public Map<String, CameraConfigurationJson> readCameraSystem(File directory)
			throws IOException, StreamReadException, DatabindException {
		Map<String, CameraConfigurationJson> camerasMap = new HashMap<>();
		for (int i = 0; i < cameras.length; i++) {
			File cameraFile = new File(directory, "cameras/" + cameras[i]);
			assert cameraFile.exists();
			CameraConfigurationJson camera = new ObjectMapper()
					.readValue(cameraFile, CameraConfigurationJson.class);
			camerasMap.put(camera.name, camera);
		}
		return camerasMap;
	}
}
