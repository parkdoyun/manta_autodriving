/* eslint-disable prettier/prettier */
import React from 'react';
import {View, StyleSheet} from 'react-native';
import {Text} from 'react-native-paper';
import {theme} from './theme';

export default function TextInput({title, data}) {
  return (
    <View style={styles.container}>
      <Text variant="titleMedium" style={{color: '#0B537F'}}>
        {title}
      </Text>
      <Text variant="titleLarge" style={styles.outline}>
        {data}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    width: '90%',
    height: 55,
    borderWidth: 2,
    borderColor: '#0B537F',
    borderRadius: 20,
    alignSelf: 'center',
    marginTop: 20,
    padding: 0,
  },
  outline: {
    width: '85%',
    borderWidth: 0,
    paddingLeft: 8,
  },
});
