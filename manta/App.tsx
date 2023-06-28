/* eslint-disable prettier/prettier */
/* eslint-disable jsx-quotes */
/* eslint-disable eol-last */
import React, { useEffect } from 'react';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import { createMaterialBottomTabNavigator } from '@react-navigation/material-bottom-tabs';
import { Alert, PermissionsAndroid } from 'react-native';
import messaging from '@react-native-firebase/messaging';
import {
  // Simple,
  SelectIndex,
  NowCarIndex,
} from './src/screen/map';
import {
  CalendarView,
} from './src/screen/calendar';
import {
  LoginScreen,
  RegisterScreen,
} from './src/screen/login';
import {
  MyInfoScreen,
  ChangeMyInfoScreen,
} from './src/screen/my';
PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.POST_NOTIFICATIONS);
messaging().setBackgroundMessageHandler(async remoteMessage => {
  console.log('[Background Remote Message]', remoteMessage);
});
const Main = createNativeStackNavigator();
const Tab = createMaterialBottomTabNavigator();
const My = createNativeStackNavigator();

const MyNavigator = () => {
  return (
    <My.Navigator initialRouteName="마이">
      <My.Screen name="마이"
        component={MyInfoScreen}
        options={{ headerShown: false }}
      />
      <My.Screen name="프로필수정"
        component={ChangeMyInfoScreen}
        options={{ headerShown: false }}
      />
    </My.Navigator>
  )
}

const TabNavigator = () => {
  return (<Tab.Navigator initialRouteName="지도"
    activeColor="#F1FFAB"
    inactiveColor="white"
    barStyle={{ backgroundColor: '#0B537F' }}

  >
    <Tab.Screen
      name="지도"
      component={NowCarIndex}
      options={{
        tabBarLabel: '지도',
        // tabBarShowIcon: false,
      }}
    />
    <Tab.Screen
      name="등하원수정"
      component={SelectIndex}
      options={{
        tabBarLabel: '등하원수정',
      }}
    />
    <Tab.Screen
      name="조회"
      component={CalendarView}
      options={{
        tabBarLabel: '조회',
      }}
    />
    <Tab.Screen
      name="마이페이지"
      component={MyNavigator}
      options={{
        tabBarLabel: '마이페이지',
      }}
    />
  </Tab.Navigator>)
};

export default function App() {
  useEffect(() => {
    const unsubscribe = messaging().onMessage(async remoteMessage => {
      const title = remoteMessage?.notification?.title; // 승차인지 하차인지
      const body = remoteMessage?.notification?.body; // 위치 및 시간
      const name = remoteMessage?.data?.name; // 원생명
      Alert.alert(title,
        '[' + name + ']' + body,
        [
          {
            text: "확인",
            onPress: () => console.log("[DIALOG] CHECK")
          }
        ]);
      console.log("[foreground] " + name + " => " + title + " : " + body);
    });

    return unsubscribe;
  }, []);
  return (
    // <MyInfoScreen />
    <NavigationContainer>
      <Main.Navigator initialRouteName='로그인'>
        <Main.Screen
          name='로그인'
          component={LoginScreen}
          options={{ headerShown: false }} />
        <Main.Screen
          name='회원가입'
          component={RegisterScreen}
          options={{ headerShown: false }} />
        <Main.Screen
          name='Main'
          component={TabNavigator}
          options={{ headerShown: false }} />
      </Main.Navigator>
    </NavigationContainer>
  );
}