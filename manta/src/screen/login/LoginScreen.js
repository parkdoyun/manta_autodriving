/* eslint-disable prettier/prettier */
/* eslint-disable handle-callback-err */
import React, {useState, useEffect} from 'react';
import {TouchableOpacity, StyleSheet, View, Image} from 'react-native';
import {Text} from 'react-native-paper';
import {Button, TextInput} from '../../components';
import AsyncStorage from '@react-native-async-storage/async-storage';
import database from '@react-native-firebase/database';
import messaging from '@react-native-firebase/messaging';

export default function LoginScreen({navigation}) {
  const [id, setId] = useState('');
  const [password, setPassword] = useState('');
  async function setFCMToken() {
    const fcmToken = await messaging().getToken();
    database()
      .ref('/' + id)
      .set({
        token: fcmToken,
      })
      .then(() => console.log('TOKEN [' + fcmToken + '] DB register SUCCESS!'))
      .catch(error => console.log('error', error));
  }
  useEffect(() => {
    var requestOptions = {
      method: 'GET',
      redirect: 'follow',
    };
    fetch('http://j8a409.p.ssafy.io/select_station_info.php', requestOptions)
      .then(response => response.text())
      .then(res => {
        AsyncStorage.setItem('bus_stop', res);
      })
      .catch(error => console.log('error', error));

    fetch(
      'http://j8a409.p.ssafy.io/select_kindergarten_info.php',
      requestOptions,
    )
      .then(response => response.text())
      .then(res => {
        AsyncStorage.setItem('kindergartens', res);
      })
      .catch(error => console.log('error', error));
  }, []);

  const getData = async result => {
    const st = {id: 0, name: '', lat: 0, lon: 0};
    const en = {id: 0, name: '', lat: 0, lon: 0};
    const kind = {id: 0, name: '', lat: 0, lon: 0};
    await AsyncStorage.getItem('bus_stop', (err, res) => {
      const bs = JSON.parse(res);
      bs.forEach(data => {
        if (result.station_in === data.id) {
          st.id = data.id;
          st.name = data.name;
          st.lat = data.lat;
          st.lon = data.lon;
        }
        if (result.station_out === data.id) {
          en.id = data.id;
          en.name = data.name;
          en.lat = data.lat;
          en.lon = data.lon;
        }
      });
    });
    await AsyncStorage.getItem('kindergartens', (err, res) => {
      const kg = JSON.parse(res);
      kg.forEach(data => {
        if (result.kindergarten_id === data.id) {
          kind.id = data.id;
          kind.name = data.name;
          kind.lat = data.lat;
          kind.lon = data.lon;
        }
      });
    });
    await AsyncStorage.setItem(
      'info',
      JSON.stringify({
        id: id,
        password: result.pwd,
        name: result.name,
        kindergarten: kind,
        tel: result.tel,
        img: result.img,
        in: st,
        out: en,
      }),
    );
    setFCMToken();
    navigation.reset({routes: [{name: 'Main'}]});
  };

  const onLoginPressed = () => {
    var formdata = new FormData();
    formdata.append('id', id);
    formdata.append('pwd', password);

    var requestOptions = {
      method: 'POST',
      body: formdata,
      redirect: 'follow',
    };

    fetch('http://j8a409.p.ssafy.io/login.php', requestOptions)
      .then(response => response.text())
      .then(res => {
        const result = JSON.parse(res);
        console.log(result);
        if (result.yn) {
          formdata = new FormData();
          formdata.append('id', id);

          requestOptions = {
            method: 'POST',
            body: formdata,
            redirect: 'follow',
          };
          fetch('http://j8a409.p.ssafy.io/select_profile.php', requestOptions)
            .then(response => response.text())
            .then(response => {
              const data = JSON.parse(response);
              getData(data);
            })
            .catch(error => console.log('error', error));
          // navigation.navigate('Main', {screen: '지도'});
        } else {
          alert('아이디가 없거나 비밀번호가 틀립니다');
        }
      })
      .catch(error => console.log('error', error));
  };

  return (
    <View style={{height: '100%', justifyContent: 'center'}}>
      <Image
        source={require('../../image/manta.png')}
        style={{
          width: 350,
          height: 300,
          resizeMode: 'cover',
          alignSelf: 'center',
        }}
      />
      <TextInput
        data="ID"
        placeholder="아이디를 입력하세요"
        returnKeyType="next"
        value={id}
        onChangeText={text => setId(text)}
        style={{width: '90%'}}
      />
      <TextInput
        data="PW"
        placeholder="비밀번호를 입력하세요"
        returnKeyType="done"
        value={password}
        onChangeText={text => setPassword(text)}
        secureTextEntry
        style={{width: '90%'}}
      />
      <Button onPress={onLoginPressed} buttonColor="#0B537F">
        로 그 인
      </Button>

      <TouchableOpacity onPress={() => navigation.navigate('회원가입')}>
        <Text style={styles.link}> 회원가입 </Text>
      </TouchableOpacity>
    </View>
  );
}

const styles = StyleSheet.create({
  row: {
    flexDirection: 'row',
    marginTop: 4,
  },
  link: {
    fontWeight: 'bold',
    color: '#4980AD',
    alignSelf: 'center',
    marginTop: 20,
    fontSize: 20,
  },
  button: {
    width: '40%',
    alignSelf: 'center',
    marginTop: 20,
    borderRadius: 15,
  },
});
