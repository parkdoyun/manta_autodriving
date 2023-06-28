/* eslint-disable prettier/prettier */
import React, {useEffect, useState, useRef} from 'react';
import {SafeAreaView, View, StyleSheet, Image} from 'react-native';
import {Text, ProgressBar} from 'react-native-paper';
import MapView, {PROVIDER_GOOGLE, Marker, Polyline} from 'react-native-maps';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function NowCarIndex() {
  const [state, setState] = useState(false);
  const [info, setInfo] = useState({});
  const [bus, setBus] = useState({lat: 0, lon: 0});
  const [toBus, setToBus] = useState(-1);
  const [busStop, setBusStop] = useState({});
  const [stopIdx, setStopIdx] = useState(['']);
  const [myIdx, setMyIdx] = useState(-1);
  const [node, setNode] = useState([
    {
      latitude: 37.2396288611,
      longitude: 126.773421955,
    },
  ]);
  const [count, setCount] = useState(0);
  function useInterval(callback, delay) {
    const savedCallback = useRef();
    useEffect(() => {
      savedCallback.current = callback;
    }, [callback]);

    useEffect(() => {
      function tick() {
        savedCallback.current();
      }
      if (delay !== null) {
        let id = setInterval(tick, delay);
        return () => clearInterval(id);
      }
    }, [delay]);
  }
  useInterval(() => {
    setCount(count => count + 1);
    var requestOptions = {
      method: 'GET',
      redirect: 'follow',
    };
    fetch('http://j8a409.p.ssafy.io/now_bus_info.php', requestOptions)
      .then(response => response.text())
      .then(res => {
        const result = JSON.parse(res);
        console.log(result);
        setBus({lat: Number(result.bus_lat), lon: Number(result.bus_lon)});
        setToBus(Number(result.station_ID));
        setState(!state);
      })
      .catch(error => console.log('error', error));
  }, 1000);
  useEffect(() => {
    async function start() {
      await AsyncStorage.getItem('info', (err, result) => {
        const UserInfo = JSON.parse(result);
        setInfo(UserInfo);
      });

      await AsyncStorage.getItem('bus_stop', (err, res) => {
        const result = JSON.parse(res);
        setBusStop({});
        result.forEach(data => {
          const num = Number(data.id);
          const name = data.name;
          setBusStop(pre => ({
            ...pre,
            [num]: name,
          }));
        });
      });
      var requestOptions = {
        method: 'GET',
        redirect: 'follow',
      };

      await fetch('http://j8a409.p.ssafy.io/now_bus_info.php', requestOptions)
        .then(response => response.text())
        .then(res => {
          const result = JSON.parse(res);
          setBus({lat: Number(result.bus_lat), lon: Number(result.bus_lon)});
          setToBus(Number(result.station_ID));
        })
        .catch(error => console.log('error', error));

      await fetch('http://j8a409.p.ssafy.io/select_node.php', requestOptions)
        .then(response => response.text())
        .then(res => {
          const result = JSON.parse(res);
          setNode([]);
          result.forEach(data => {
            setNode(pre => [
              ...pre,
              {
                latitude: Number(data.lat),
                longitude: Number(data.lon),
              },
            ]);
          });
          setNode(pre => [
            ...pre,
            {
              latitude: Number(result[0].lat),
              longitude: Number(result[0].lon),
            },
          ]);
        })
        .catch(error => console.log('error', error));
    }
    start();
  }, []);
  useEffect(() => {
    var requestOptions = {
      method: 'GET',
      redirect: 'follow',
    };
    fetch(
      'http://j8a409.p.ssafy.io/select_route_all_station.php',
      requestOptions,
    )
      .then(response => response.text())
      .then(res => {
        const result = JSON.parse(res);
        setStopIdx(['']);
        result.forEach(idx => {
          setStopIdx(pre => [...pre, busStop[idx.station_id]]);
          if (idx.station_id === info.in.id) {
            setMyIdx(Number(idx.idx));
          }
        });
      })
      .catch(error => console.log('error', error));
  }, [busStop]);

  return (
    <SafeAreaView style={{width: '100%', height: '100%'}}>
      <MapView
        style={{width: '100%', height: '100%', zIndex: -1}}
        provider={PROVIDER_GOOGLE}
        mapType="standard"
        zoom="20"
        // zoomEnabled='true'
        region={{
          latitude: bus.lat,
          longitude: bus.lon,
          latitudeDelta: 0.0922,
          longitudeDelta: 0.0421,
        }}>
        <Marker
          coordinate={{
            latitude: bus.lat,
            longitude: bus.lon,
          }}>
          <Image
            source={require('../../image/marker.png')}
            resizeMode="contain"
            style={{width: 50, height: 60}}
          />
        </Marker>
        <Polyline
          coordinates={node}
          strokeColor="#0B537F" // fallback for when `strokeColors` is not supported by the map-provider
          strokeColors={[
            '#7F0000',
            '#00000000', // no color, creates a "long" gradient between the previous and next coordinate
            '#B24112',
            '#E5845C',
            '#238C23',
            '#7F0000',
          ]}
          strokeWidth={8}
        />
      </MapView>
      <View style={styles.container}>
        <View
          style={{
            flexDirection: 'row',
            width: '90%',
            flex: 1,
            alignItems: 'center',
            justifyContent: 'center',
          }}>
          <View style={{flex: 3}}>
            <Text
              variant="headlineLarge"
              style={{color: '#0B537F', fontWeight: 'bold'}}>
              오는 중이에요
            </Text>
            <Text variant="titleMedium" style={{color: '#7AD6CC'}}>
              {info.name}이(가) 안전하게 오고 있어요
            </Text>
          </View>
          <View
            style={{
              backgroundColor: '#F1FFAB',
              borderRadius: 20,
              alignItems: 'center',
              justifyContent: 'center',
              flex: 1,
              padding: 10,
            }}>
            <Text variant="titleMedium" style={{color: '#0B537F'}}>
              남은 정류장
            </Text>
            <Text
              variant="displayMedium"
              style={{color: '#0B537F', fontWeight: 'bold'}}>
              {myIdx - toBus > 0 ? myIdx - toBus : 0}
            </Text>
          </View>
        </View>
        <View
          style={{
            flex: 1,
            alignItems: 'center',
            justifyContent: 'center',
            width: '100%',
          }}>
          <View
            style={{
              flexDirection: 'row',
              alignItems: 'center',
              justifyContent: 'center',
            }}>
            <Text>출발</Text>
            <ProgressBar
              progress={toBus / myIdx}
              color="#0B537F"
              style={{width: 300}}
            />
            <Text>도착</Text>
          </View>
          {toBus !== -1 && <Text>{stopIdx[toBus]}를 향해 이동중 입니다.</Text>}
        </View>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    position: 'absolute',
    left: 10,
    top: 10,
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 30,
    width: '95%',
    height: '30%',
    backgroundColor: 'white',
    borderRadius: 20,
  },
  separator: {
    width: 3,
  },
});
