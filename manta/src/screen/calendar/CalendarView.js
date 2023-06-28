/* eslint-disable prettier/prettier */
/* eslint-disable react-native/no-inline-styles */
import React, {useState, useEffect} from 'react';
import {format} from 'date-fns';
import {CalendarList} from 'react-native-calendars';
import {SafeAreaView, View, StyleSheet, Pressable} from 'react-native';
import {Text, Modal} from 'react-native-paper';
import {Simple} from '../map';
import AsyncStorage from '@react-native-async-storage/async-storage';

function CalendarView() {
  const [id, setId] = useState('');
  const [visible, setVisible] = useState('');
  const [index, setIndex] = useState({lat: 0, lon: 0});
  const [posts, setPosts] = useState([]);
  useEffect(() => {
    AsyncStorage.getItem('info', (err, result) => {
      const UserInfo = JSON.parse(result);
      setId(UserInfo.id);
    });
  }, []);
  useEffect(() => {
    console.log(id);
    if (id !== '') {
      var formdata = new FormData();
      formdata.append('id', id);

      var requestOptions = {
        method: 'POST',
        body: formdata,
        redirect: 'follow',
      };

      fetch('http://j8a409.p.ssafy.io/select_history.php', requestOptions)
        .then(response => response.text())
        .then(res => {
          const result = JSON.parse(res);
          setPosts([]);
          for (let i = 0; i < result.length; i++) {
            setPosts(pre => [...pre, result[i]]);
          }
        })
        .catch(error => console.log('error', error));
    }
  }, [id]);

  const showModal = state => setVisible(state);
  const hideModal = () => setVisible('');
  const markedDates = posts.reduce((acc, current) => {
    const formattedDate = format(new Date(current.time), 'yyyy-MM-dd');
    if (acc[formattedDate]) acc[formattedDate].data.push(current);
    else acc[formattedDate] = {marked: true, data: [current]};
    return acc;
  }, {});

  const [selectedDate, setSelectedDate] = useState(
    format(new Date(), 'yyyy-MM-dd'),
  );
  const markedSelectedDates = {
    ...markedDates,
    [selectedDate]: {
      selected: true,
      marked: markedDates[selectedDate]?.marked,
    },
  };

  return (
    <SafeAreaView style={{width: '100%', height: '100%'}}>
      <CalendarList
        // Enable horizontal scrolling, default = false
        horizontal={true}
        // Enable paging on horizontal, default = false
        pagingEnabled={true}
        // Set custom calendarWidth.
        style={styles.calendar}
        markedDates={markedSelectedDates}
        theme={{
          selectedDayBackgroundColor: '#009688',
          arrowColor: '#009688',
          dotColor: '#009688',
          todayTextColor: '#009688',
        }}
        onDayPress={day => {
          setSelectedDate(day.dateString);
        }}
      />
      {markedDates[selectedDate] &&
        markedDates[selectedDate].data.map(e => (
          <Pressable
            style={{margin: 20}}
            key={e.in_out}
            onPress={() => {
              showModal(e.in_out === '0' ? 'start' : 'end');
              setIndex({lat: Number(e.lat), lon: Number(e.lon)});
            }}>
            <Text variant="headlineSmall">{e.time}</Text>
            <View style={styles.container}>
              <Text variant="displaySmall" style={{flex: 3}}>
                {e.station_name}
              </Text>
              <Text
                variant="displayMedium"
                style={{
                  color: e.in_out === '0' ? '#D8EC75' : '#7AD6CC',
                  fontWeight: 'bold',
                  flex: 1,
                }}>
                {e.in_out === '0' ? '등원' : '하원'}
              </Text>
            </View>
          </Pressable>
        ))}
      <Modal
        visible={visible}
        onDismiss={hideModal}
        contentContainerStyle={styles.modal}>
        <Simple lat={index.lat} lon={index.lon} state={visible} />
      </Modal>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  calendar: {
    borderBottomWidth: 1,
    borderBottomColor: '#e0e0e0',
  },
  container: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  text: {
    fontWeight: 'bold',
  },
  modal: {
    backgroundColor: 'white',
    padding: 20,
    width: '100%',
    height: '50%',
  },
});

export default CalendarView;
