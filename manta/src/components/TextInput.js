/* eslint-disable prettier/prettier */
import React from 'react';
import {View, StyleSheet, Text} from 'react-native';
import {TextInput as Input} from 'react-native-paper';
import {theme} from './theme';

export default function TextInput({
  data,
  placeholder,
  errorText,
  description,
  ...props
}) {
  return (
    <View style={styles.container}>
      <Text variant="titleMedium" style={{color: '#0B537F'}}>
        {data}
      </Text>
      <Input
        style={styles.input}
        placeholder={placeholder}
        underlineColor="transparent"
        mode="outlined"
        activeOutlineColor="#0B537F"
        outlineStyle={styles.outline}
        {...props}
      />
      {description && !errorText ? (
        <Text style={styles.description}>{description}</Text>
      ) : null}
      {errorText ? <Text style={styles.error}>{errorText}</Text> : null}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
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
  input: {
    // backgroundColor: 'white',
    padding: 0,
    margin: 0,
  },
  outline: {
    // margin: 16,
    height: 50,
    width: '90%',
    borderWidth: 0,
  },
  description: {
    fontSize: 13,
    color: 'white',
    // paddingTop: 8,
  },
  error: {
    fontSize: 13,
    color: theme.colors.error,
    paddingTop: 8,
  },
});
