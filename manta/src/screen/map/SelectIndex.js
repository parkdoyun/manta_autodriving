/* eslint-disable handle-callback-err */
/* eslint-disable prettier/prettier */
/* eslint-disable react-hooks/exhaustive-deps */
import React, {useState, useEffect} from 'react';
import {
  ScrollView,
  SafeAreaView,
  View,
  StyleSheet,
  Pressable,
  Alert,
} from 'react-native';
import {Text} from 'react-native-paper';
import MapView, {PROVIDER_GOOGLE, Marker} from 'react-native-maps';
import {format} from 'date-fns';
import ko from 'date-fns/esm/locale/ko/index.js';
import DateTimePickerModal from 'react-native-modal-datetime-picker';
import {Button, Dropdown} from '../../components';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function SelectIndex({func, inIndex, outIndex}) {
  const [id, setId] = useState('');
  const [busStopInfo, setBusStopInfo] = useState([]);
  const [startIndex, setStartIndex] = useState({
    name: '',
    lat: 0,
    lon: 0,
    id: 0,
  });
  const [startDate, setStartDate] = useState('0000-01-01');
  const [endIndex, setEndIndex] = useState({name: '', lat: 0, lon: 0, id: 0});
  const [endDate, setEndDate] = useState('0000-01-01');
  const [visible, setVisible] = useState(''); // 모달 노출 여부
  const [idx, setIdx] = useState({
    lat: 37.243091121949796,
    lon: 126.77451930691763,
  });
  useEffect(() => {
    AsyncStorage.getItem('info', (err, result) => {
      const UserInfo = JSON.parse(result);
      setId(UserInfo.id);
    });
    AsyncStorage.getItem('bus_stop', (err, res) => {
      const result = JSON.parse(res);
      setBusStopInfo([]);
      result.forEach(data => {
        setBusStopInfo(pre => [
          ...pre,
          {
            label: data.name,
            value: data,
          },
        ]);
      });
    });
  }, []);
  useEffect(() => {
    if (inIndex === undefined && outIndex === undefined) {
      AsyncStorage.getItem('info', (err, res) => {
        const result = JSON.parse(res);
        setStartIndex(result.in);
        setEndIndex(result.out);
      });
    } else {
      setStartIndex(inIndex);
      setEndIndex(outIndex);
    }
  }, [func, inIndex, outIndex]);

  const onPressDate = state => {
    // 날짜 클릭 시
    setVisible(state); // 모달 open
  };

  const onConfirm = selectedDate => {
    // 날짜 또는 시간 선택 시
    const select = new Date(selectedDate);
    const year = select.getFullYear();
    const month = select.getMonth() + 1;
    const date = select.getDate();
    const change = `${year}-${month >= 10 ? month : '0' + month}-${
      date >= 10 ? date : '0' + date
    } 00:00:00`;
    if (visible === 'start') setStartDate(change); // 선택한 날짜 변경
    else setEndDate(change);
    setVisible(''); // 모달 close
  };

  const onCancel = () => {
    // 취소 시
    setVisible(''); // 모달 close
  };
  const send = async () => {
    if (func !== undefined) {
      func({in: startIndex, out: endIndex});
    } else {
      if (startDate === '0000-01-01' && endDate === '0000-01-01') return;
      var formdata = new FormData();
      var success1 = startDate === '0000-01-01';
      var success2 = endDate === '0000-01-01';
      if (startDate !== '0000-01-01') {
        formdata.append('id', id);
        formdata.append('in_out', '0');
        formdata.append('station_id', Number(startIndex.id));
        formdata.append('time', startDate);
        var requestOptions = {
          method: 'POSt',
          body: formdata,
          redirect: 'follow',
        };
        await fetch(
          'http://j8a409.p.ssafy.io/update_position.php',
          requestOptions,
        )
          .then(response => response.text())
          .then(result => {
            success1 = true;
          })
          .catch(error => console.log('error', error));
      }
    }
    if (endDate !== '0000-01-01') {
      formdata = new FormData();
      formdata.append('id', id);
      formdata.append('in_out', '1');
      formdata.append('station_id', Number(endIndex.id));
      formdata.append('time', endDate);
      var requestOptions = {
        method: 'POST',
        body: formdata,
        redirect: 'follow',
      };
      await fetch(
        'http://j8a409.p.ssafy.io/update_position.php',
        requestOptions,
      )
        .then(response => response.text())
        .then(result => {
          success2 = true;
        })
        .catch(error => console.log('error', error));
    }
    if (success1 && success2) {
      AsyncStorage.getItem('info', (err, result) => {
        const info = JSON.parse(result);
        AsyncStorage.setItem(
          'info',
          JSON.stringify({
            id: info.id,
            password: info.pwd,
            name: info.name,
            kindergarten: info.kindergarten,
            tel: info.tel,
            img: info.img,
            in: startIndex,
            out: endIndex,
          }),
        );
      });
      Alert.alert('일일 위치 변경', '위치 변경 되었습니다.');
    }
  };
  return (
    <SafeAreaView style={{width: '100%', height: '100%'}}>
      <MapView
        style={{width: '100%', height: '50%'}}
        provider={PROVIDER_GOOGLE}
        mapType="standard"
        // zoomEnabled='true'
        region={{
          latitude: idx.lat,
          longitude: idx.lon,
          latitudeDelta: 0.0922,
          longitudeDelta: 0.0421,
        }}>
        {startIndex.name !== '' && (
          <Marker
            coordinate={{
              latitude: Number(startIndex.lat),
              longitude: Number(startIndex.lon),
            }}
            image={require('../../image/start.png')}
            style={{width: 10, height: 10}}
          />
        )}
        {endIndex.name !== '' && (
          <Marker
            coordinate={{
              latitude: Number(endIndex.lat),
              longitude: Number(endIndex.lon),
            }}
            image={require('../../image/end.png')}
            style={{width: 10, height: 10}}
          />
        )}
      </MapView>
      <ScrollView>
        <View style={styles.container}>
          <Text variant="titleLarge" style={styles.text}>
            등원
          </Text>
          <View style={{flex: 6}}>
            {func === undefined && (
              <Pressable
                onPress={() => onPressDate('start')}
                style={styles.dateContainer}>
                <Text style={styles.text}>
                  {startDate === '0000-01-01'
                    ? '날짜를 선택하세요'
                    : format(new Date(startDate), 'PPP', {locale: ko})}
                </Text>
              </Pressable>
            )}
            <Dropdown
              style={{backgroundColor: '#F1FFAB'}}
              data={busStopInfo}
              placeholder={
                startIndex.name === '' ? '등원 위치선택' : startIndex.name
              }
              value={startIndex}
              onChange={item => {
                setStartIndex(item.value);
              }}
            />
          </View>
        </View>
        <View style={styles.container}>
          <Text variant="titleLarge" style={styles.text}>
            하원
          </Text>
          <View style={{flex: 6}}>
            {func === undefined && (
              <Pressable
                onPress={() => onPressDate('end')}
                style={styles.dateContainer}>
                <Text style={styles.text}>
                  {endDate === '0000-01-01'
                    ? '날짜를 선택하세요'
                    : format(new Date(endDate), 'PPP', {locale: ko})}
                </Text>
              </Pressable>
            )}
            <Dropdown
              style={{backgroundColor: '#7AD6CC'}}
              data={busStopInfo}
              placeholder={
                endIndex.name === '' ? '하원 위치선택' : endIndex.name
              }
              value={endIndex}
              onChange={item => {
                setEndIndex(item.value);
              }}
            />
          </View>
        </View>
        <Button buttonColor="#0B537F" onPress={send}>
          지정
        </Button>
      </ScrollView>
      <DateTimePickerModal
        isVisible={visible !== ''}
        mode="date"
        onConfirm={onConfirm}
        onCancel={onCancel}
      />
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flexDirection: 'row',
    alignSelf: 'center',
    alignItems: 'center',
    marginTop: 20,
    width: '80%',
  },
  dateContainer: {
    height: 50,
    width: '100%',
    borderRadius: 20,
    borderColor: '#0B537F',
    borderWidth: 2,
    padding: 12,
    marginBottom: 5,
  },
  text: {
    color: '#0B537F',
    fontWeight: 'bold',
    flex: 1,
  },
  separator: {
    width: 3,
  },
});
