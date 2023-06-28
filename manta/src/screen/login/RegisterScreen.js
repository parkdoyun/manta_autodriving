/* eslint-disable prettier/prettier */
import React, {useEffect, useState} from 'react';
import {
  View,
  StyleSheet,
  TouchableOpacity,
  ScrollView,
  Alert,
} from 'react-native';
import {Text, Avatar, HelperText, Modal} from 'react-native-paper';
import {launchImageLibrary, launchCamera} from 'react-native-image-picker';
import UploadModeModal from './UploadModeModal';
import {
  passwordValidator,
  checkPasswordValidator,
} from '../../components/AuthValidator';
import {Button, Dropdown, TextInput} from '../../components';
import {Simple, SelectIndex} from '../map';
import AsyncStorage from '@react-native-async-storage/async-storage';

const imagePickerOption = {
  mediaType: 'photo',
  maxWidth: 768,
  maxHeight: 768,
  includeBase64: true,
};

export default function RegisterScreen({navigation}) {
  const [modalVisible, setModalVisible] = useState(false);
  const [kindergartens, setKindergartens] = useState([]);
  const [checkPassword, setCheckPassword] = useState('');
  const [visible, setVisible] = useState('');
  const [Data, setData] = useState({
    id: '',
    password: '',
    name: '',
    kindergarten: {id: '0', name: '', lat: '0', lon: '0'},
    tel: '',
    img: 'undefined',
    in: {id: '0', name: '', lat: '0', lon: '0'},
    out: {id: '0', name: '', lat: '0', lon: '0'},
  });
  useEffect(() => {
    AsyncStorage.getItem('kindergartens', (err, res) => {
      const result = JSON.parse(res);
      setKindergartens([]);
      result.forEach(data => {
        setKindergartens(pre => [
          ...pre,
          {
            label: data.name,
            value: data,
          },
        ]);
      });
    });
  }, []);

  const showModal = state => setVisible(state);
  const hideModal = () => setVisible('');

  const onSubmit = () => {
    const passwordError = passwordValidator(Data.password);
    const checkPasswordError = checkPasswordValidator(
      Data.password,
      checkPassword,
    );
    if (passwordError || checkPasswordError) return;
    if (
      Data.id === '' ||
      Data.password === '' ||
      Data.name === '' ||
      Data.kindergarten.id === '' ||
      Data.tel === '' ||
      Data.img === 'undefined' ||
      Data.in.id === '0' ||
      Data.out.id === '0'
    ) {
      Alert.alert('회원가입 실패', '입력안된 정보가 있습니다.');
      return;
    }
    var formdata = new FormData();
    formdata.append('id', Data.id);
    formdata.append('pwd', Data.password);
    formdata.append('name', Data.name);
    formdata.append('kindergarten_id', Number(Data.kindergarten.id));
    formdata.append('tel', Data.tel);
    formdata.append('img', Data.img);
    formdata.append('station_in', Number(Data.in.id));
    formdata.append('station_out', Number(Data.out.id));

    var requestOptions = {
      method: 'POST',
      body: formdata,
      redirect: 'follow',
    };

    fetch('http://j8a409.p.ssafy.io/sign_info.php', requestOptions)
      .then(response => response.text())
      .then(res => {
        const result = JSON.parse(res);
        console.log(result);
        if (result.yn) {
          Alert.alert('회원가입', `${Data.name}님회원가입 되었습니다.`);
          navigation.reset({routes: [{name: '로그인'}]});
        }
      })
      .catch(error => console.log('error', error));
  };
  const onPickImage = res => {
    if (res.didCancel || !res) {
      return;
    }
    const base = res.assets[0].base64;
    setData(pre =>
      Object.assign({}, pre, {img: `data:image/jpg;base64,${base}`}),
    );
  };
  // 카메라 촬영
  const onLaunchCamera = () => {
    launchCamera(imagePickerOption, onPickImage);
  };

  // 갤러리에서 사진 선택
  const onLaunchImageLibrary = () => {
    launchImageLibrary(imagePickerOption, onPickImage);
  };
  const modalOpen = () => {
    setModalVisible(true);
  };

  const getIndex = data => {
    setData(pre => Object.assign({}, pre, {in: data.in}));
    setData(pre => Object.assign({}, pre, {out: data.out}));
    setVisible('');
  };
  return (
    <ScrollView>
      <View style={{flexDirection: 'row', alignSelf: 'center', marginTop: 20}}>
        <Avatar.Image
          size={150}
          source={
            Data.img === 'undefined'
              ? require('../../image/1.png')
              : {uri: Data.img}
          }
          style={{backgroundColor: '#fff', marginRight: 20}}
        />
        <Button
          mode="contained"
          onPress={modalOpen}
          buttonColor="#F1FFAB"
          textColor="#0B537F">
          사진등록
        </Button>
      </View>
      <TextInput
        data="ID"
        placeholder="아이디를 입력하세요"
        returnKeyType="next"
        value={Data.id}
        onChangeText={text =>
          setData(pre => Object.assign({}, pre, {id: text}))
        }
        autoCapitalize="none"
        style={{width: '90%'}}
      />
      <Button
        mode="contained"
        buttonColor="#0B537F"
        onPress={async () => {
          var formdata = new FormData();
          formdata.append('id', Data.id);

          var requestOptions = {
            method: 'POST',
            body: formdata,
            redirect: 'follow',
          };

          fetch('http://j8a409.p.ssafy.io/id_chk.php', requestOptions)
            .then(response => response.text())
            .then(res => {
              const result = JSON.parse(res);
              if (result.yn) {
                Alert.alert('아이디 중복 확인', '사용 가능한 아이디입니다.');
              } else {
                setData(pre => Object.assign({}, pre, {id: ''}));
                Alert.alert('아이디 중복 확인', '사용할 수 없는 아이디입니다.');
              }
            })
            .catch(error => console.log('error', error));
        }}>
        중복확인
      </Button>
      <TextInput
        data="PW"
        placeholder="비밀번호를 입력하세요"
        returnKeyType="next"
        value={Data.password}
        onChangeText={text =>
          setData(pre => Object.assign({}, pre, {password: text}))
        }
        secureTextEntry
        onEndEditing={() => {
          switch (passwordValidator(Data.password)) {
            case `Password can't be empty.`:
              alert('비밀번호를 입력하세요!');
              break;
            case `Password must be at least 8 characters.`:
              alert('8글자 이상 17글자 미만으로 입력해주세요.');
              break;
          }
        }}
        style={{width: '90%'}}
      />
      <TextInput
        data="확인"
        placeholder="위와 같은 비밀번호를 입력하세요"
        returnKeyType="done"
        value={checkPassword}
        onChangeText={text => setCheckPassword(text)}
        secureTextEntry
        style={{width: '90%'}}
      />
      {Data.password !== '' && Data.password === checkPassword && (
        <HelperText
          type="info"
          style={{
            justifyContent: 'center',
            alignItems: 'center',
            width: '90%',
            color: '#199B64',
            marginLeft: 25,
          }}>
          비밀번호가 일치합니다.
        </HelperText>
      )}
      {Data.password !== checkPassword && (
        <HelperText
          type="error"
          style={{
            justifyContent: 'center',
            alignItems: 'center',
            width: '90%',
            marginLeft: 25,
          }}>
          비밀번호가 일치하지 않습니다.
        </HelperText>
      )}
      <TextInput
        data="이름"
        placeholder="홍길동"
        returnKeyType="done"
        value={Data.name}
        onChangeText={text =>
          setData(pre => Object.assign({}, pre, {name: text}))
        }
        style={{width: '90%'}}
      />
      <Dropdown
        style={{
          backgroundColor: 'white',
          width: '90%',
          alignSelf: 'center',
          marginTop: 20,
        }}
        data={kindergartens}
        placeholder={
          Data.kindergarten.name === '' ? '유치원 선택' : Data.kindergarten.name
        }
        value={Data.kindergarten}
        onChange={item => {
          setData(pre => Object.assign({}, pre, {kindergarten: item.value}));
          showModal('kindergarten');
        }}
      />
      <TextInput
        data="전화"
        placeholder="010-1234-5678"
        returnKeyType="done"
        value={Data.tel}
        onChangeText={text =>
          setData(pre => Object.assign({}, pre, {tel: text}))
        }
        style={{width: '90%'}}
      />
      <View
        style={{
          flexDirection: 'row',
          alignSelf: 'center',
          width: '90%',
          justifyContent: 'space-between',
        }}>
        <Button
          mode="contained"
          onPress={() => setVisible('index')}
          buttonColor="#F1FFAB"
          textColor="#0B537F">
          승하차 지정
        </Button>
        <Button mode="contained" onPress={onSubmit} buttonColor="#0B537F">
          회원가입
        </Button>
      </View>
      <View style={styles.row}>
        <Text>Already have an account?</Text>
        <TouchableOpacity
          onPress={() => navigation.reset({routes: [{name: '로그인'}]})}>
          <Text style={styles.link}> Login </Text>
        </TouchableOpacity>
      </View>
      <UploadModeModal
        visible={modalVisible}
        onClose={() => setModalVisible(false)}
        onLaunchCamera={onLaunchCamera}
        onLaunchImageLibrary={onLaunchImageLibrary}
      />
      <Modal
        visible={visible === 'kindergarten'}
        onDismiss={hideModal}
        contentContainerStyle={styles.modal}>
        <Simple
          lat={Number(Data.kindergarten.lat)}
          lon={Number(Data.kindergarten.lon)}
          state={visible}
        />
      </Modal>
      <Modal
        visible={visible === 'index'}
        onDismiss={hideModal}
        contentContainerStyle={[styles.modal, {height: '80%'}]}>
        <SelectIndex func={getIndex} inIndex={Data.in} outIndex={Data.out} />
      </Modal>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  row: {
    flexDirection: 'row',
    marginTop: 4,
  },
  link: {
    fontWeight: 'bold',
    color: '#0B537F',
  },
  inputContainer: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    width: '90%',
    borderWidth: 2,
    borderColor: '#0B537F',
    borderRadius: 20,
    alignSelf: 'center',
    marginTop: 20,
    padding: 0,
  },
  modal: {
    backgroundColor: 'white',
    padding: 20,
    width: '100%',
    height: '50%',
  },
});
