/* eslint-disable handle-callback-err */
/* eslint-disable prettier/prettier */
/* eslint-disable react-hooks/rules-of-hooks */
import React, {useState, useEffect} from 'react';
import {View, StyleSheet, ScrollView, Pressable} from 'react-native';
import {Avatar, Modal} from 'react-native-paper';
import {Button, Text} from '../../components';
import {Simple} from '../map';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function MyInfoScreen({navigation}) {
  const [visible, setVisible] = useState({id: 0, name: '', lat: 0, lon: 0});
  const [Data, setData] = useState({
    id: '',
    password: '3',
    name: '',
    kindergarten: {id: 0, name: '', lat: 0, lon: 0},
    tel: '',
    img: 'undefined',
    in: {id: 0, name: '', lat: 0, lon: 0},
    out: {id: 0, name: '', lat: 0, lon: 0},
  });
  useEffect(() => {
    var info = {};
    AsyncStorage.getItem('info', (err, result) => {
      info = JSON.parse(result);
      setData(info);
    });
  }, []);

  const hideModal = () => setVisible({id: 0, name: '', lat: 0, lon: 0});

  return (
    <ScrollView style={{Height: '100%'}}>
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
          onPress={() => navigation.reset({routes: [{name: '프로필수정'}]})}
          buttonColor="#F1FFAB"
          textColor="#0B537F">
          정보수정
        </Button>
      </View>
      <Text title="이름" data={Data.name} />
      <Pressable onPress={() => setVisible(Data.kindergarten)}>
        <Text title="소속" data={Data.kindergarten.name} />
      </Pressable>
      <Text title="전화" data={Data.tel} />
      <Pressable onPress={() => setVisible(Data.in)}>
        <Text title="등원지" data={Data.in.name} />
      </Pressable>
      <Pressable onPress={() => setVisible(Data.out)}>
        <Text title="하원지" data={Data.out.name} />
      </Pressable>
      <Modal
        visible={visible.name !== ''}
        onDismiss={hideModal}
        contentContainerStyle={styles.modal}>
        <Simple lat={visible.lat} lon={visible.lon} state={''} />
      </Modal>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  modal: {
    backgroundColor: 'white',
    padding: 20,
    width: '100%',
    height: '50%',
  },
});
