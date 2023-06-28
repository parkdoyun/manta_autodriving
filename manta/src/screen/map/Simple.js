/* eslint-disable prettier/prettier */
import React from 'react';
import {SafeAreaView, StyleSheet, Image} from 'react-native';
import MapView, {PROVIDER_GOOGLE, Marker} from 'react-native-maps';

export default function Simple({lat, lon, state}) {
  return (
    <SafeAreaView style={styles.container}>
      <MapView
        style={styles.container}
        provider={PROVIDER_GOOGLE}
        mapType="standard"
        // zoomEnabled='true'
        region={{
          latitude: lat,
          longitude: lon,
          latitudeDelta: 0.0922,
          longitudeDelta: 0.0421,
        }}>
        <Marker
          coordinate={{
            latitude: lat,
            longitude: lon,
          }}>
          <Image
            source={
              state === 'start'
                ? require('../../image/start.png')
                : state === 'end'
                ? require('../../image/end.png')
                : require('../../image/marker.png')
            }
            resizeMode="contain"
            style={{width: 30, height: 30}}
          />
        </Marker>
      </MapView>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    width: '100%',
    height: '100%',
  },
});
